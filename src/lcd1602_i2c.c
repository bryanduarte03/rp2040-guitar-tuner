#include "i2c_lcd1602.h"
#include "pico/stdlib.h"
#include <string.h>

// PCF8574 -> LCD bit mapping (most common backpacks):
// P0=RS, P1=RW, P2=E, P3=Backlight, P4..P7 = D4..D7
#define PIN_RS 0x01
#define PIN_RW 0x02
#define PIN_E  0x04
#define PIN_BL 0x08

static inline uint8_t bl_mask(const lcd1602_i2c_t *lcd) {
    return lcd->backlight ? PIN_BL : 0;
}

static void expander_write(const lcd1602_i2c_t *lcd, uint8_t data) {
    // Blocking write, no STOP issues
    i2c_write_blocking(lcd->i2c, lcd->addr, &data, 1, false);
}

static void pulse_enable(const lcd1602_i2c_t *lcd, uint8_t data) {
    expander_write(lcd, data | PIN_E);
    sleep_us(1);
    expander_write(lcd, data & ~PIN_E);
    sleep_us(50);
}

static void write4bits(const lcd1602_i2c_t *lcd, uint8_t nibble_with_ctrl) {
    expander_write(lcd, nibble_with_ctrl);
    pulse_enable(lcd, nibble_with_ctrl);
}

static void send(const lcd1602_i2c_t *lcd, uint8_t value, uint8_t mode_rs) {
    uint8_t hi = (value & 0xF0);
    uint8_t lo = (value << 4) & 0xF0;

    write4bits(lcd, hi | mode_rs | bl_mask(lcd));
    write4bits(lcd, lo | mode_rs | bl_mask(lcd));
}

static void command(const lcd1602_i2c_t *lcd, uint8_t cmd) {
    send(lcd, cmd, 0);
    if (cmd == 0x01 || cmd == 0x02) sleep_ms(2); // clear/home need longer
}

static void write_char(const lcd1602_i2c_t *lcd, char c) {
    send(lcd, (uint8_t)c, PIN_RS);
}

void lcd1602_init(lcd1602_i2c_t *lcd, i2c_inst_t *i2c, uint8_t addr, bool backlight) {
    lcd->i2c = i2c;
    lcd->addr = addr;
    lcd->backlight = backlight;

    sleep_ms(50);

    // 8-bit init sequence (nibbles only)
    write4bits(lcd, 0x30 | bl_mask(lcd)); sleep_ms(5);
    write4bits(lcd, 0x30 | bl_mask(lcd)); sleep_us(150);
    write4bits(lcd, 0x30 | bl_mask(lcd)); sleep_us(150);

    // switch to 4-bit
    write4bits(lcd, 0x20 | bl_mask(lcd)); sleep_us(150);

    // function set: 4-bit, 2 line, 5x8
    command(lcd, 0x28);
    // display on, cursor off, blink off
    command(lcd, 0x0C);
    // entry mode: increment, no shift
    command(lcd, 0x06);
    // clear
    command(lcd, 0x01);
}

void lcd1602_clear(lcd1602_i2c_t *lcd) { command(lcd, 0x01); }
void lcd1602_home(lcd1602_i2c_t *lcd)  { command(lcd, 0x02); }

void lcd1602_set_cursor(lcd1602_i2c_t *lcd, int col, int row) {
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row < 0) row = 0;
    if (row > 1) row = 1;
    if (col < 0) col = 0;
    if (col > 15) col = 15;
    command(lcd, 0x80 | (uint8_t)(col + row_offsets[row]));
}

void lcd1602_write_str(lcd1602_i2c_t *lcd, const char *s) {
    while (*s) write_char(lcd, *s++);
}
