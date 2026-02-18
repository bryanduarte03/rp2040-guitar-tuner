#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"

//settings
#define ADC_PIN        26
#define ADC_CH         0

#define FS_HZ          10000.0f
#define N              2048

#define NOTE_HOLD_MS       180
#define PRINT_MIN_MS       120

#define AMP_ON             80
#define AMP_OFF            55

#define CONF_RATIO         1.7f

#define ATTACK_SKIP_MS      120

// Cents smoothing
#define CENTS_EMA_ALPHA     0.12f   // smaller = smoother
#define IN_TUNE_CENTS       15.0f    // within ±4c => IN

//LEDs 
#define LED_GREEN_PIN 15
#define LED_RED_PIN   14

//I2C LCD 1602 
#define I2C_PORT       i2c0
#define I2C_SDA_PIN    4
#define I2C_SCL_PIN    5
#define LCD_ADDR       0x27
#define I2C_BAUD       50000

// PCF8574 mapping: P0=RS P1=RW P2=E P3=BL P4..P7=D4..D7
#define PIN_RS 0x01
#define PIN_E  0x04
#define PIN_BL 0x08

static bool lcd_backlight = true;

static inline uint8_t lcd_bl_mask(void) { 
    return lcd_backlight ? PIN_BL : 0; }

static void lcd_expander_write(uint8_t data) {
    i2c_write_blocking(I2C_PORT, LCD_ADDR, &data, 1, false);
}

static void lcd_pulse_enable(uint8_t data) {
    lcd_expander_write(data | PIN_E);
    sleep_us(1);
    lcd_expander_write((uint8_t)(data & ~PIN_E));
    sleep_us(50);
    }

static void lcd_write4bits(uint8_t nibble_with_ctrl) {
    lcd_expander_write(nibble_with_ctrl);
    lcd_pulse_enable(nibble_with_ctrl);
}

static void lcd_send(uint8_t value, bool rs_mode) {
    uint8_t high = value & 0xF0;
    uint8_t low  = (uint8_t)((value << 4) & 0xF0);
    uint8_t ctrl = lcd_bl_mask() | (rs_mode ? PIN_RS : 0);
    lcd_write4bits(high | ctrl);
    lcd_write4bits(low  | ctrl);
 }

static void lcd_command(uint8_t cmd) { 
    lcd_send(cmd, false); 
    }

    static void lcd_write_char(char c)   { 
    lcd_send((uint8_t)c, true); 
    }

    static void lcd_clear(void) { 
    lcd_command(0x01); sleep_ms(2); 
    }

static void lcd_set_cursor(int col, int row) {
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row < 0) row = 0;
    if (row > 1) row = 1;
    if (col < 0) col = 0;
    if (col > 15) col = 15;
    lcd_command((uint8_t)(0x80 | (row_offsets[row] + (uint8_t)col)));
}
static void lcd_write_line(int row, const char *text16) {
    char buf[17];
    size_t len = strlen(text16);
    if (len > 16) {
        len = 16;
    }
    memset(buf, ' ', 16);
    memcpy(buf, text16, len);
    buf[16] = '\0';

    lcd_set_cursor(0, row);
    for (int i = 0; i < 16; i++) {
        lcd_write_char(buf[i]);
    }
}
static void lcd_init(void) {
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(50);

    uint8_t ctrl = lcd_bl_mask();
    lcd_write4bits(0x30 | ctrl); sleep_ms(5);
    lcd_write4bits(0x30 | ctrl); sleep_us(150);
    lcd_write4bits(0x30 | ctrl); sleep_us(150);
    lcd_write4bits(0x20 | ctrl); sleep_us(150);

    lcd_command(0x28);
    lcd_command(0x08);
    lcd_clear();
    lcd_command(0x06);
    lcd_command(0x0C);
}

// guitar notes
static const char *notes[6]  = {"E2","A2","D3","G3","B3","E4"};
static const float freqs[6]  = {82.41f,110.00f,146.83f,196.00f,246.94f,329.63f};

// ADC + DMA
static uint16_t adc_buf[N];
static int dma_chan;
static float win[N];

static void wait_for_usb(void) {
    sleep_ms(1200);
    absolute_time_t start = get_absolute_time();

    while (!stdio_usb_connected()) {
        if (absolute_time_diff_us(start, get_absolute_time()) > 8 * 1000 * 1000){ 
            break;
        }

        sleep_ms(20);
    }
}

static void build_hann_window(void) {
    for (int i = 0; i < N; i++) {
        win[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * 
        (float)i / (float)(N - 1)));
    }
}

static void adc_dma_init(void) {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_CH);

    adc_fifo_setup(true, true, 1, false, false);
    adc_fifo_drain();
    adc_set_clkdiv((48000000.0f / FS_HZ) - 1.0f);

    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    dma_channel_configure(dma_chan, &c, adc_buf, &adc_hw->fifo, N, false);
}

static void capture_block(void) {
    adc_fifo_drain();
    adc_run(false);

    dma_channel_set_read_addr(dma_chan, &adc_hw->fifo, false);
    dma_channel_set_write_addr(dma_chan, adc_buf, false);
    dma_channel_set_trans_count(dma_chan, N, true);

    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);
}

//goertzel and refinement
static float goertzel_power_k(const float *x, int n, int k) {
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;
    float w = 2.0f * (float)M_PI * (float)k / (float)n;
    float coeff = 2.0f * cosf(w);

    for (int i = 0; i < n; i++) {
        s0 = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    return s1*s1 + s2*s2 - coeff*s1*s2;
}

static float goertzel_power_freq(const float *x, int n, float f, float fs) {
    float kf = (n * f) / fs;
    int k = (int)(kf + 0.5f);
    
    if (k < 1) {
        k = 1;
    }
    
    if (k > n - 2) {
        k = n - 2;
    }
    
    return goertzel_power_k(x, n, k);
}

// Quadratic interpolation around nearest bin (k-1,k,k+1)
static float refine_freq_hz(const float *x, int n, float fs, float f_note) {
    float kf = (n * f_note) / fs;
    int k0 = (int)(kf + 0.5f);
    
    if (k0 < 1) {
        k0 = 1;
    }
    
    if (k0 > n - 2) {
        k0 = n - 2;
    }

    float pm = goertzel_power_k(x, n, k0 - 1);
    float p0 = goertzel_power_k(x, n, k0);
    float pp = goertzel_power_k(x, n, k0 + 1);
    float denom = (pm - 2.0f*p0 + pp);
    float delta = 0.0f;
    
    if (fabsf(denom) > 1e-12f) {
        delta = 0.5f * (pm - pp) / denom;
    }

    float k_est = (float)k0 + delta;
    
    return (k_est * fs) / (float)n;
}

static float cents_off(float f_est, float f_note) {
    return 1200.0f * (logf(f_est / f_note) / logf(2.0f));
}

//preprocess
static void prep_block(float *x_out, uint16_t *mn_out, uint16_t *mx_out) {
    uint16_t mn = adc_buf[0] & 0x0FFF;
    uint16_t mx = mn;
    float mean = 0.0f;

    for (int i = 0; i < N; i++) {
        uint16_t s = adc_buf[i] & 0x0FFF;
        if (s < mn) mn = s;
        if (s > mx) mx = s;
        mean += (float)s;
    }

    mean /= (float)N;
    const float scale = 1.0f / 2048.0f;

    for (int i = 0; i < N; i++) {
        float v = (float)(adc_buf[i] & 0x0FFF) - mean;
        x_out[i] = (v * scale) * win[i];
    }

    *mn_out = mn;
    *mx_out = mx;
}

// score = F + a bit of 2F ONLY (NO 3F -> fixes A2->E4)
static float note_score(const float *x, float f0) {
    float nyq = FS_HZ * 0.5f;
    float p1 = goertzel_power_freq(x, N, f0, FS_HZ);
    float p2 = (2.0f*f0 < nyq) ? goertzel_power_freq(x, N, 2.0f*f0, FS_HZ) : 0.0f;
    float comb = p1 + 0.35f * p2;

    return comb / f0;
}

// median-of-5 for cents -spikes
static inline void sort2(float *a, float *b) { if (*a > *b) { float t=*a; *a=*b; *b=t; } }
static float median5(float a, float b, float c, float d, float e) {
    sort2(&a,&b); 
    sort2(&d,&e);
    sort2(&a,&c); 
    sort2(&b,&c);
    sort2(&a,&d); 
    sort2(&c,&d);
    sort2(&b,&e); 
    sort2(&b,&c);
    sort2(&c,&d);
    return c;
}

int main(void) {
    stdio_init_all();
    wait_for_usb();

    // LEDs
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, 0);

    // LCD
    lcd_init();
    lcd_write_line(0, "Guitar Tuner");
    lcd_write_line(1, "Listening");

    build_hann_window();
    adc_dma_init();

    float x[N];

    bool active = false;
    absolute_time_t active_since = get_absolute_time();
    absolute_time_t stable_since = get_absolute_time();
    absolute_time_t last_out = get_absolute_time();

    int prev_candidate = -1;
    int last_note = -1;
    float last_cents = 9999.0f;

    // cents filters
    float c_hist[5] = {0};
    int c_hist_n = 0;
    int c_hist_i = 0;
    float c_ema = 0.0f;
    bool c_ema_valid = false;

    while (true) {
        capture_block();

        uint16_t mn, mx;
        prep_block(x, &mn, &mx);

        int amp = (int)mx - (int)mn;

        // silence gating and LED off
        if (!active) {
            if (amp >= AMP_ON) {
                active = true;
                active_since = get_absolute_time();
                prev_candidate = -1;
                stable_since = get_absolute_time();
                c_hist_n = 0; c_hist_i = 0;
                c_ema_valid = false;
            } else {
                gpio_put(LED_GREEN_PIN, 0);
                gpio_put(LED_RED_PIN, 0);
                continue;
            }
        } 
        else {
            if (amp <= AMP_OFF) {
                active = false;
                gpio_put(LED_GREEN_PIN, 0);
                gpio_put(LED_RED_PIN, 0);
                lcd_write_line(0, "Guitar Tuner");
                lcd_write_line(1, "Listening...");
                continue;
            }
        }

        // attack skip
        if (absolute_time_diff_us(active_since, get_absolute_time()) < (ATTACK_SKIP_MS * 1000)) {
            continue;
        }

        // fundamental powers (for harmonic sanity check)
        float pFund[6];
        for (int i = 0; i < 6; i++) {
            pFund[i] = goertzel_power_freq(x, N, freqs[i], FS_HZ);
        }

        // pick best/second by score
        int best = 0, second = 1;
        float best_s = -1.0f, second_s = -1.0f;
        for (int i = 0; i < 6; i++) {
            float s = note_score(x, freqs[i]);
            if (s > best_s) {
                second_s = best_s; second = best;
                best_s = s; best = i;
            } else if (s > second_s) {
                second_s = s; second = i;
            }
        }

        // confidence
        if (!(best_s > (CONF_RATIO * second_s))) continue;

        // Harmonic sanity check: prefer lower string if winner is its 2x/3x/4x harmonic
        const float SUB_RATIO = 0.60f;
        int corrected = best;
        for (int j = 0; j < 6; j++) {
            if (j == best) continue;
            for (int m = 2; m <= 4; m++) {
                float harmonic = freqs[j] * (float)m;
                float rel = fabsf(harmonic - freqs[best]) / freqs[best];
                if (rel < 0.02f) {
                    if (pFund[j] > (SUB_RATIO * pFund[best])) corrected = j;
                }
            }
        }
        best = corrected;

        // note hold
        if (best != prev_candidate) {
            prev_candidate = best;
            stable_since = get_absolute_time();
            continue;
        }
        if (absolute_time_diff_us(stable_since, get_absolute_time()) < (NOTE_HOLD_MS * 1000)) {
            continue;
        }

        // estimate freq/cents refine near expected note
        float f_note = freqs[best];
        float f_est  = refine_freq_hz(x, N, FS_HZ, f_note);
        float c_raw  = cents_off(f_est, f_note);

        bool note_changed = (best != last_note);

        // cents median-of-5 then EMA
        c_hist[c_hist_i] = c_raw;
        c_hist_i = (c_hist_i + 1) % 5;
        if (c_hist_n < 5) c_hist_n++;

        float c_med = c_raw;
        if (c_hist_n == 5) {
            c_med = median5(c_hist[0], c_hist[1], c_hist[2], c_hist[3], c_hist[4]);
        }

        if (note_changed || !c_ema_valid) {
            c_ema = c_med;
            c_ema_valid = true;
        } else {
            c_ema = (CENTS_EMA_ALPHA * c_med) + ((1.0f - CENTS_EMA_ALPHA) * c_ema);
        }

        float cents = c_ema;
        float abs_c = fabsf(cents);

        // label: show IN immediately when within window
        const char *label = (abs_c <= IN_TUNE_CENTS) ? "IN" : ((cents > 0.0f) ? "SHARP" : "FLAT");

        // LEDs match label
        bool in_now = (abs_c <= IN_TUNE_CENTS);
        gpio_put(LED_GREEN_PIN, in_now ? 1 : 0);
        gpio_put(LED_RED_PIN,   in_now ? 0 : 1);

        // update outputs (USB + LCD) with throttle
        bool change = note_changed || (fabsf(cents - last_cents) >= 1.0f);
        if (change && absolute_time_diff_us(last_out, get_absolute_time()) >= (PRINT_MIN_MS * 1000)) {
            printf("amp=%d  note=%s  est=%.2fHz  cents=%+0.1f  %s\n",
                   amp, notes[best], f_est, cents, label);

            char line0[32], line1[32];
            snprintf(line0, sizeof(line0), "%-2s %+.1fc %-5s", notes[best], cents, label);
            snprintf(line1, sizeof(line1), "%5.1fHz a%4d", f_est, amp);
            lcd_write_line(0, line0);
            lcd_write_line(1, line1);

            last_out = get_absolute_time();
            last_note = best;
            last_cents = cents;
        }
    }
}
