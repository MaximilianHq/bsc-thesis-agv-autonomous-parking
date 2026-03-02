#pragma once
#include <Arduino.h>

struct Packet
{
    char type;             // type
    size_t data_len = 0;   // data_length
    uint8_t data[9] = {0}; // data
    uint8_t crc = 0;       // checksum
    bool approved = false; // data integrity
};

uint8_t csum(const Packet &pkt);
void read_bt(Packet &out);
bool write_bt(const Packet &pkt);