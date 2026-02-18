#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;      
    bool backlight;    // true = on
} lcd1602_i2c_t;

void lcd1602_init(lcd1602_i2c_t *lcd, i2c_inst_t *i2c, uint8_t addr, bool backlight);
void lcd1602_clear(lcd1602_i2c_t *lcd);
void lcd1602_home(lcd1602_i2c_t *lcd);
void lcd1602_set_cursor(lcd1602_i2c_t *lcd, int col, int row); // row 0/1, col 0..15
void lcd1602_write_str(lcd1602_i2c_t *lcd, const char *s);
