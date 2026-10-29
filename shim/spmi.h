#pragma once
#include <stdint.h>

/* 0 on success, else the channel status bits (2 fail, 4 denied, 8 dropped), -1 no channel, -2 timeout */
int spmi_read(uint8_t sid, uint16_t addr, uint8_t *val);
int spmi_write(uint8_t sid, uint16_t addr, uint8_t val);
