#pragma once
#include "types.h"
#include <Arduino.h>
#include <BluetoothSerial.h>

uint8_t csum(const Packet &pkt);
void read_bt(BluetoothSerial &str, Packet &out);
bool write_bt(BluetoothSerial &str, const Packet &pkt);