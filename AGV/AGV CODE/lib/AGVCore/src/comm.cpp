#include <Arduino.h>
#include <types.h>
#include "comm.h"

namespace
{
    void print_hex_byte(uint8_t value)
    {
        if (value < 0x10)
            Serial.print("0");
        Serial.print(value, HEX);
    }

    void debug_print_packet(const String &dbg_name, const char *label, const Comm::Packet &pkt, uint8_t crc_value)
    {
        Serial.print("[COMM ");
        Serial.print(dbg_name);
        Serial.print("] ");
        Serial.print(label);
        Serial.print(": ");

        print_hex_byte(static_cast<uint8_t>('$'));
        Serial.print(" ");
        print_hex_byte(static_cast<uint8_t>(pkt.type));

        for (size_t i = 0; i < pkt.data_len; i++)
        {
            Serial.print(" ");
            print_hex_byte(pkt.data[i]);
        }

        Serial.print(" ");
        print_hex_byte(pkt.seq);
        Serial.print(" ");
        print_hex_byte(crc_value);
        Serial.print(" ");
        print_hex_byte(static_cast<uint8_t>('\n'));
        Serial.println();

        Serial.print("[COMM ");
        Serial.print(dbg_name);
        Serial.print("] type=");
        Serial.write(pkt.type);
        Serial.print(" data_len=");
        Serial.print(pkt.data_len);
        Serial.print(" seq=0x");
        print_hex_byte(pkt.seq);
        Serial.print(" crc=0x");
        print_hex_byte(crc_value);
        Serial.print(" calc=0x");
        print_hex_byte(Comm::csum(pkt));
        Serial.print(" approved=");
        Serial.println(pkt.approved ? "yes" : "no");
    }
} // namespace

uint8_t Comm::csum(const Packet &pkt)
{
    uint8_t crc = 0;
    crc ^= static_cast<uint8_t>(pkt.type);
    for (size_t i = 0; i < pkt.data_len; i++)
        crc ^= pkt.data[i];
    crc ^= pkt.seq;
    return crc;
}

Comm::Comm(Stream &s, const String &debug_id) : _str(s), _dbg_name(debug_id) {}

bool Comm::read(Packet &out)
{
    bool started_packet = false;

    while (_str.available())
    {
        uint8_t b = static_cast<uint8_t>(_str.read());

        switch (_state)
        {
        case WAIT_START:
            if (b == '$')
            {
                started_packet = true;
                _pkt = Packet{};
                _read_index = 0;
                _state = READ_TYPE;
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Packet recieving...");
            }
            break;

        case READ_TYPE:
            if (b == '$')
            {
                _pkt = Packet{};
                _read_index = 0;
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Restarting packet read");
                break;
            }

            if (b == '\r' || b == '\n')
            {
                _state = WAIT_START;
                break;
            }

            _pkt.type = static_cast<char>(b);
            _state = READ_BODY;
            break;

        case READ_BODY:
            if (b == '$')
            {
                _pkt = Packet{};
                _read_index = 0;
                _state = READ_TYPE;
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Resync on new start byte");
                break;
            }

            if (b == '\r')
                break;

            if (b == '\n')
            {
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Packet recieved!");
                if (_read_index >= 2)
                {
                    _pkt.data_len = _read_index - 2; // points to seq
                    _pkt.seq = _pkt.data[_pkt.data_len];
                    _pkt.crc = _pkt.data[_pkt.data_len + 1];
                    _pkt.data[_pkt.data_len] = 0;
                    _pkt.data[_pkt.data_len + 1] = 0;
                    _pkt.approved = (_pkt.crc == csum(_pkt));

                    if (g_debug.comm)
                        debug_print_packet(_dbg_name, "Recieved frame", _pkt, _pkt.crc);

                    out = _pkt;
                    _state = WAIT_START;
                    return true;
                }

                _state = WAIT_START;
            }
            else
            {
                if (_read_index < sizeof(_pkt.data))
                {
                    _pkt.data[_read_index++] = b;
                }
                else
                {
                    // overflow: kasta paket och vänta på ny start
                    if (g_debug.comm)
                        Serial.println("[COMM " + _dbg_name + "] Packet corrupt");
                    _state = WAIT_START;
                }
            }
            break;
        }
    }

    if (g_debug.comm && started_packet)
        Serial.println("[COMM " + _dbg_name + "] Read paused");

    return false;
}

bool Comm::write(const Packet &pkt)
{
    if (!pkt.approved)
        return false;

    if (g_debug.comm)
        debug_print_packet(_dbg_name, "Sending frame", pkt, (pkt.crc != 0) ? pkt.crc : csum(pkt));

    _str.write(static_cast<uint8_t>('$'));
    _str.write(static_cast<uint8_t>(pkt.type));

    _str.write(pkt.data, pkt.data_len);
    _str.write(pkt.seq);
    _str.write((pkt.crc != 0) ? pkt.crc : csum(pkt));

    _str.write(static_cast<uint8_t>('\n'));

    return true;
}

ProtocolHandler::ProtocolHandler(Comm &comm, SysCtrl &actions) : _comm(comm), _actions(actions) {}

uint8_t ProtocolHandler::get_sequence() const { return _seq; }
void ProtocolHandler::iterate_sequence()
{
    uint8_t prev = _seq;
    _seq++;
    if (prev > _seq && g_debug.comm)
        Serial.println("[PROTOHANDLER] \033[33mWARNING\033[0m - Sequence overflow");
}

void ProtocolHandler::add_buffer_sent(const Comm::Packet &pkt) { _pkt_buffer_sent.push_back(pkt); }
void ProtocolHandler::add_buffer_rcvd(const Comm::Packet &pkt) { _pkt_buffer_rcvd.push_back(pkt); }

void ProtocolHandler::handle(const Comm::Packet &pkt)
{
    // Handle asnwers
    if (pkt.type == 'A')
    {
        if (pkt.approved)
            _pkt_buffer_sent.pop_if([&pkt](const Comm::Packet &p)
                                    { return p.seq == pkt.seq; });
        else
            _comm.write(*_pkt_buffer_sent.find_if([&pkt](const Comm::Packet &p)
                                                  { return p.seq == pkt.seq; }));

        return;
    }

    // Handle commands
    auto ans = _make_ack(pkt);
    if (_comm.write(ans))
    {
        if (g_debug.comm)
            Serial.println("[PROTOHANDLER] Sent response succesfully");
    }
    else if (g_debug.comm)
        Serial.println("[PROTOHANDLER] Failed to respond");

    if (pkt.approved)
        _pkt_buffer_rcvd.push_back(pkt);
}

Comm::Packet ProtocolHandler::_make_ack(const Comm::Packet &pkt)
{
    Comm::Packet p = {'A', pkt.seq, {((pkt.approved) ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00))}, 1, 0, true};
    p.crc = Comm::csum(p);
    return p;
}
