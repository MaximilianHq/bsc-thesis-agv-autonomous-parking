#pragma once
#include <Arduino.h>
#include <types.h>
#include <static_vector.h>

class SysCtrl;

class Comm
{
public:
    static constexpr size_t DATA_BYTES = 10;
    static constexpr size_t FRAME_BYTES = 14;

    struct Packet
    {
        char type;             // type
        uint8_t seq = 0;       // sequence
        uint8_t data[DATA_BYTES] = {0}; // data
        uint8_t data_len = 0;  // data_length
        uint8_t crc = 0;       // checksum
        bool approved = false; // data integrity
    };

    static uint8_t csum(const Comm::Packet &pkt);

    Comm(Stream &s, const String &debug_id);

    bool read(Packet &out);
    bool write(const Packet &pkt);

private:
    Stream &_str;

    enum ParserState
    {
        WAIT_START,
        READ_TYPE,
        READ_BODY
    };

    ParserState _state = WAIT_START;
    Packet _pkt;
    size_t _read_index = 0;

    const String _dbg_name;
};

class ProtocolHandler
{
public:
    ProtocolHandler(Comm &comm, SysCtrl &actions);

    struct Sequence
    {
        uint8_t seq;
        bool avalible;
    };

    Sequence get_sequence() const;
    void handle(const Comm::Packet &pkt);
    bool send_pkt(Comm::Packet &pkt, bool que = true);
    void clear_send_queue();

private:
    static Comm::Packet _make_ack(const Comm::Packet &pkt);

    Comm &_comm;
    SysCtrl &_actions;

    uint8_t _seq;
    bool _avalible = true;

    StaticVector<Comm::Packet, 10> _pkt_buffer_sent;
    StaticVector<Comm::Packet, 10> _pkt_send_que;

    void _iterate_sequence();
};
