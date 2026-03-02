#pragma once
#include <Arduino.h>

void ota_begin(const char *ssid, const char *password);
void ota_handle();