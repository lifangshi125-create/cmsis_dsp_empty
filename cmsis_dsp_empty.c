/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arm_math.h"
#include "ti_msp_dl_config.h"

/* ---------------------------------------------------------------------------
 * MSPM0G3507 board driving a ZDT / Emm42 style closed loop stepper driver
 *
 *   UART_DRIVER : UART0 on PA0 (TX) / PA1 (RX), 115200 8N1, going to the
 *                 serial port of the driver board (the pins marked A0/A1):
 *                   PA0 (TX) -> driver RX
 *                   PA1 (RX) <- driver TX
 *                   GND      <-> driver GND          (this one is mandatory)
 *                 PA0/PA1 are OPEN-DRAIN pads - they can pull low, never high.
 *                 Measure PA0 against GND while idle: near 3.3V means a
 *                 pull-up is present and all is well, a low reading means a
 *                 10k resistor from PA0 to 3.3V has to be added.
 *   UART_DEBUG  : UART1 on PA8 (TX) / PA9 (RX), 115200 8N1. Optional - delete
 *                 the instance in the .syscfg if those pins are not broken out
 *                 on the board. A USB-TTL adapter on PA8/PA9 shows the trace
 *                 printed below.
 *   BUTTONS     : PB25 and PB22, meant for the two external buttons, plus
 *                 PB21 as a spare input - all wired pin-to-GND. Press any one
 *                 of them to start the motor at low speed, press again to
 *                 stop it.
 *   SERVO       : PA27 outputs a 50 Hz servo signal (TIMG8). The servo turned
 *                 out to be a continuous rotation type: 1.5 ms stops it,
 *                 away from 1.5 ms it spins, faster the further away. At
 *                 power up it spins at full speed for SERVO_RUN_MS (5 s)
 *                 and then stops. Set SERVO_TEST to 0 to disable it.
 *   LCD         : 1.44" ST7735 (128x128) on SPI1 - SCLK = PB9, PICO = PB8,
 *                 RES = PB10, DC = PB11, CS = PB0 (held low), BLK = PB26.
 *                 At power up it flashes red/green/blue and then shows
 *                 colour bars. Set LCD_TEST to 0 to disable it.
 * ------------------------------------------------------------------------- */

/* ------------------------------ debug console ---------------------------- */
static void dbgChar(char c)
{
    DL_UART_transmitDataBlocking(UART_DEBUG_INST, (uint8_t) c);
}

static void dbgStr(const char *s)
{
    while (*s != '\0') {
        dbgChar(*s++);
    }
}

static void dbgHex(uint8_t b)
{
    const char digits[] = "0123456789ABCDEF";

    dbgChar(digits[(b >> 4) & 0x0F]);
    dbgChar(digits[b & 0x0F]);
    dbgChar(' ');
}

static void dbgFrame(const uint8_t *frame, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        dbgHex(frame[i]);
    }
}

/* -------------------------------- timing --------------------------------- */
/* delay_cycles() burns one CPU cycle per iteration, so CPUCLK_FREQ/1000
 * cycles equal 1 ms (32 MHz clock tree -> 32000 cycles per ms) */
#define DELAY_MS(ms) ((CPUCLK_FREQ / 1000) * (ms))

/* --------------------------- Emm_V5 protocol -----------------------------
 * Every frame is: address + function code + payload + checksum.
 * The checksum byte defaults to the fixed value 0x6B. Only if the "checksum"
 * entry in the driver menu is changed to sum / XOR / CRC-8 does it have to be
 * computed - if you change it there, change EMM_CHECKSUM here as well.
 */
#define EMM_ADDR           (0x01)  /* must match the address set on the board */
#define EMM_CHECKSUM       (0x6B)

#define EMM_FUNC_ENABLE    (0xF3)  /* energise / release the motor */
#define EMM_FUNC_SPEED     (0xF6)  /* run continuously at a speed */
#define EMM_FUNC_STOP      (0xFE)  /* stop immediately */

#define EMM_DIR_CW         (0x00)
#define EMM_DIR_CCW        (0x01)

#define EMM_ACCEL          (10)    /* 0..255, smaller = snappier start */
#define EMM_RPM_LOW        (10)    /* slow test speed, also used by the buttons */

/* Bring-up aid: 1 = at power up, without touching any button, run the motor
 * for five seconds at EMM_RPM_LOW (10 RPM -> about 0.8 of a turn). It
 * separates the two possible failures - if the motor turns the serial link
 * and the frames are fine and only the buttons need checking; if it stays
 * still the link itself is broken. Set to 0 afterwards. */
#define EMM_BOOT_TEST      (0)

/* Position mode is not used by this program - here is the frame in case you
 * want it later (3200 pulses = one revolution at the factory 16 microsteps):
 *   addr FD dir SPDH SPDL ACC P3 P2 P1 P0 ABS sync 6B                     */

/* Sends one frame and dumps the driver answer on the debug console. */
static void emmSend(const uint8_t *frame, uint32_t len)
{
    /* Drop stale bytes so the answer printed below belongs to this frame
     * (a position command is only acknowledged once the move is done) */
    while (!DL_UART_isRXFIFOEmpty(UART_DRIVER_INST)) {
        (void) DL_UART_receiveData(UART_DRIVER_INST);
    }

    dbgStr("TX: ");
    dbgFrame(frame, len);

    for (uint32_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART_DRIVER_INST, frame[i]);
    }

    /* Give the driver a moment to answer before looking at the RX FIFO */
    delay_cycles(DELAY_MS(2));

    dbgStr("| RX: ");
    uint32_t count = 0;

    while (!DL_UART_isRXFIFOEmpty(UART_DRIVER_INST) && (count < 16)) {
        dbgHex(DL_UART_receiveData(UART_DRIVER_INST));
        count++;
    }

    if (count == 0) {
        dbgStr("(no answer)");
    }
    dbgStr("\r\n");
}

/* 01 F3 AB 01 00 6B - 0x01 holds the motor, 0x00 releases it */
static void emmEnable(bool enable)
{
    uint8_t frame[6] = {EMM_ADDR, EMM_FUNC_ENABLE, 0xAB,
        (uint8_t)(enable ? 0x01 : 0x00), 0x00, EMM_CHECKSUM};

    emmSend(frame, sizeof(frame));
}

/* 01 F6 dir SPDH SPDL ACC 00 6B - speed in RPM, 0..3000 */
static void emmSpeed(uint8_t dir, uint16_t rpm, uint8_t accel)
{
    uint8_t frame[8] = {EMM_ADDR, EMM_FUNC_SPEED, dir, (uint8_t)(rpm >> 8),
        (uint8_t)(rpm & 0xFF), accel, 0x00, EMM_CHECKSUM};

    emmSend(frame, sizeof(frame));
}

/* 01 FE 98 00 6B */
static void emmStop(void)
{
    uint8_t frame[5] = {EMM_ADDR, EMM_FUNC_STOP, 0x98, 0x00, EMM_CHECKSUM};

    emmSend(frame, sizeof(frame));
}

/* ------------------------------- buttons ---------------------------------
 * Every switch sits between the pin and GND, so the internal pull-ups keep
 * them high and a press reads low. Only a falling edge toggles the motor and
 * the first reading never counts as a press, so an input that is low all the
 * time cannot start the motor by itself.
 */
#define BTN_PORT (GPIO_BUTTONS_PORT)
#define BTN_PINS                                                              \
    (GPIO_BUTTONS_USER_BTN_1_PIN | GPIO_BUTTONS_USER_BTN_2_PIN |              \
        GPIO_BUTTONS_USER_BTN_3_PIN)

/* True when a switch was just pressed (debounced falling edge) */
static bool startStopPressed(void)
{
    static uint32_t prev   = 0;
    static bool     primed = false;
    uint32_t        now    = DL_GPIO_readPins(BTN_PORT, BTN_PINS);
    uint32_t        falling;

    if (!primed) {
        primed = true;
        prev   = now;
        return false;
    }

    falling = (prev & ~now) & BTN_PINS; /* pins that just went low */

    if (falling != 0) {
        delay_cycles(DELAY_MS(20)); /* let the contact stop bouncing */
        now  = DL_GPIO_readPins(BTN_PORT, BTN_PINS);
        prev = now;

        if ((falling & ~now) != 0) { /* still low -> a real press */
            return true;
        }
    }

    prev = now;
    return false;
}

/* ------------------------------ servo test -------------------------------
 * Hobby servo signal on PA27: 50 Hz, 1.0 ms .. 2.0 ms pulse. The PWM timer
 * runs at 2 MHz (one tick = 0.5 us) and counts down from SERVO_PERIOD, so the
 * compare value is SERVO_PERIOD minus the pulse width in ticks.
 */
#define SERVO_PERIOD        (40000) /* 40000 * 0.5 us = 20 ms -> 50 Hz */
#define SERVO_STOP_US       (1500)  /* centre pulse -> no rotation */
#define SERVO_DRIVE_US      (2000)  /* full speed in one direction */

/* This servo is a continuous rotation type: pulse width sets speed and
 * direction (1.5 ms stops it), there is no position control. The test below
 * simply spins it at full speed for SERVO_RUN_MS and then stops it. */
#define SERVO_RUN_MS        (5000)
#define SERVO_TEST          (0)     /* 1 = sweep the servo (uses the PWM timer) */

#if SERVO_TEST
static void servoSetPulse(uint32_t microseconds)
{
    uint32_t ticks = microseconds * 2; /* 2 MHz -> 0.5 us per tick */

    if (ticks >= SERVO_PERIOD) {
        ticks = SERVO_PERIOD - 1;
    }

    DL_TimerG_setCaptureCompareValue(
        PWM_SERVO_INST, SERVO_PERIOD - ticks, DL_TIMER_CC_1_INDEX);
}
#endif

/* ------------------------------- ST7735 LCD -------------------------------
 * 1.44" 128x128 panel on SPI1: SCLK = PB9, PICO (MOSI) = PB8, plus the four
 * control lines RES/DC/CS/BLK as plain GPIOs. The panel is write-only, so the
 * SPI runs PICO-only with no hardware chip select and CS is held low.
 */
#define LCD_W        (128)
#define LCD_H        (128)
/* The visible area starts a few rows into the controller RAM. Garbage along
 * the bottom edge means YOFF is too small; garbage at the top means it is too
 * big. Common values: 2/1, 2/3, 0/0. */
#define LCD_XOFF     (2)
#define LCD_YOFF     (3)
#define LCD_TEST     (0) /* colour test at power up */
#define IMU_TEST     (0) /* I2C bus scan at power up */
#define FLUID_TEST   (1) /* FLIP water demo, tilt controlled */
#define GYRO_TEST    (0) /* IMU963RA gyro readout on the LCD */
/* The LCD driver is needed by these, keep it compiled if any of them runs */
#define LCD_ENABLED  (LCD_TEST || IMU_TEST || FLUID_TEST || GYRO_TEST)

#if LCD_ENABLED

#define LCD_RGB565(r, g, b)                                                   \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

static void lcdWaitIdle(void)
{
    while (DL_SPI_isBusy(SPI_LCD_INST)) {
    }
}

static void lcdCmd(uint8_t cmd)
{
    DL_GPIO_clearPins(GPIO_TFT_PORT, GPIO_TFT_DC_PIN); /* command */
    DL_SPI_transmitDataBlocking8(SPI_LCD_INST, cmd);
    lcdWaitIdle();
}

static void lcdDat(const uint8_t *data, uint32_t len)
{
    DL_GPIO_setPins(GPIO_TFT_PORT, GPIO_TFT_DC_PIN); /* data */
    while (len--) {
        DL_SPI_transmitDataBlocking8(SPI_LCD_INST, *data++);
    }
    lcdWaitIdle();
}

static void lcdWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t c[4];

    x0 += LCD_XOFF;
    x1 += LCD_XOFF;
    y0 += LCD_YOFF;
    y1 += LCD_YOFF;

    lcdCmd(0x2A); /* column address range */
    c[0] = (uint8_t)(x0 >> 8);
    c[1] = (uint8_t) x0;
    c[2] = (uint8_t)(x1 >> 8);
    c[3] = (uint8_t) x1;
    lcdDat(c, 4);

    lcdCmd(0x2B); /* row address range */
    c[0] = (uint8_t)(y0 >> 8);
    c[1] = (uint8_t) y0;
    c[2] = (uint8_t)(y1 >> 8);
    c[3] = (uint8_t) y1;
    lcdDat(c, 4);
}

static void lcdRect(
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t colour)
{
    uint8_t  hi    = (uint8_t)(colour >> 8);
    uint8_t  lo    = (uint8_t) colour;
    uint32_t count = (uint32_t)(x1 - x0 + 1) * (y1 - y0 + 1);

    lcdWindow(x0, y0, x1, y1);
    lcdCmd(0x2C); /* memory write */
    DL_GPIO_setPins(GPIO_TFT_PORT, GPIO_TFT_DC_PIN);

    while (count--) {
        DL_SPI_transmitDataBlocking8(SPI_LCD_INST, hi);
        DL_SPI_transmitDataBlocking8(SPI_LCD_INST, lo);
    }
    lcdWaitIdle();
}

/* ------------------------------- text on LCD ------------------------------
 * Minimal 8x8 font with only the characters used by the tests. Each glyph is
 * eight rows, bit 7 is the leftmost pixel.
 */
typedef struct {
    char    c;
    uint8_t rows[8];
} lcd_glyph_t;

static const lcd_glyph_t lcd_font[] = {
    {'0', {0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00}},
    {'1', {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00}},
    {'2', {0x3C, 0x66, 0x06, 0x0C, 0x30, 0x60, 0x7E, 0x00}},
    {'3', {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00}},
    {'4', {0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x0C, 0x00}},
    {'5', {0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00}},
    {'6', {0x1C, 0x30, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00}},
    {'7', {0x7E, 0x66, 0x06, 0x0C, 0x18, 0x18, 0x18, 0x00}},
    {'8', {0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00}},
    {'9', {0x3C, 0x66, 0x66, 0x3E, 0x06, 0x0C, 0x38, 0x00}},
    {'A', {0x18, 0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x00}},
    {'B', {0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00}},
    {'C', {0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00}},
    {'D', {0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00}},
    {'E', {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x7E, 0x00}},
    {'F', {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x00}},
    {'G', {0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00}},
    {'H', {0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00}},
    {'O', {0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00}},
    {'R', {0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x00}},
    {'S', {0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00}},
    {'N', {0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x00}},
    {'U', {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00}},
    {'L', {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00}},
    {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00}},
    {'P', {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00}},
    {'T', {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}},
    {'W', {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00}},
    {'X', {0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00}},
    {'Y', {0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00}},
    {'Z', {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00}},
    {'-', {0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00}},
    {':', {0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00}},
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

static void lcdChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg)
{
    const uint8_t *rows = 0;

    for (uint32_t i = 0; i < sizeof(lcd_font) / sizeof(lcd_font[0]); i++) {
        if (lcd_font[i].c == c) {
            rows = lcd_font[i].rows;
            break;
        }
    }

    lcdWindow(x, y, (uint16_t)(x + 7), (uint16_t)(y + 7));
    lcdCmd(0x2C);
    DL_GPIO_setPins(GPIO_TFT_PORT, GPIO_TFT_DC_PIN);

    for (uint32_t r = 0; r < 8; r++) {
        uint8_t bits = rows ? rows[r] : 0;

        for (uint32_t b = 0; b < 8; b++) {
            uint16_t colour = (bits & (0x80 >> b)) ? fg : bg;

            DL_SPI_transmitDataBlocking8(SPI_LCD_INST, (uint8_t)(colour >> 8));
            DL_SPI_transmitDataBlocking8(SPI_LCD_INST, (uint8_t) colour);
        }
    }
    lcdWaitIdle();
}

static void lcdText(
    uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    while (*s) {
        lcdChar(x, y, *s++, fg, bg);
        x = (uint16_t)(x + 8);
    }
}

/* Signed integer, fixed width, right aligned, space padded */
static void lcdInt(uint16_t x, uint16_t y, int32_t v, uint8_t width,
    uint16_t fg, uint16_t bg)
{
    char    buf[12];
    int32_t n = 0;
    uint8_t i;

    if (v < 0) {
        buf[n++] = '-';
        v        = -v;
    }
    if (v == 0) {
        buf[n++] = '0';
    }
    while (v > 0 && n < 10) {
        buf[n++] = (char)('0' + (v % 10));
        v /= 10;
    }

    for (i = (uint8_t) n; i < width; i++) {
        lcdChar(x, y, ' ', fg, bg);
        x = (uint16_t)(x + 8);
    }
    while (n > 0) {
        lcdChar(x, y, buf[--n], fg, bg);
        x = (uint16_t)(x + 8);
    }
}

/* Unsigned value in hex, fixed number of digits */
static void lcdHex(uint16_t x, uint16_t y, uint32_t v, uint8_t digits,
    uint16_t fg, uint16_t bg)
{
    for (uint8_t i = digits; i > 0; i--) {
        uint8_t nib = (uint8_t)((v >> ((i - 1) * 4)) & 0x0F);

        lcdChar(x, y, (nib < 10) ? (char)('0' + nib) : (char)('A' + nib - 10),
            fg, bg);
        x = (uint16_t)(x + 8);
    }
}

/* Value in units of 10^-decimals, printed right aligned in a fixed field */
static void lcdFixed(uint16_t x, uint16_t y, int32_t v, uint32_t decimals,
    uint16_t width, uint16_t fg, uint16_t bg)
{
    char     buf[12];
    int32_t  n   = 0;
    bool     neg = (v < 0);
    uint32_t i;

    if (neg) {
        v = -v;
    }
    for (i = 0; i < decimals; i++) {
        buf[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    buf[n++] = '.';
    if (v == 0) {
        buf[n++] = '0';
    }
    while (v > 0 && n < 10) {
        buf[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    if (neg) {
        buf[n++] = '-';
    }

    for (i = (uint32_t) n; i < width; i++) {
        lcdChar(x, y, ' ', fg, bg);
        x = (uint16_t)(x + 8);
    }
    while (n > 0) {
        lcdChar(x, y, buf[--n], fg, bg);
        x = (uint16_t)(x + 8);
    }
}

static void lcdInit(void)
{
    /* Backlight on, chip select held low - single device on this bus */
    DL_GPIO_setPins(GPIO_TFT_PORT, GPIO_TFT_BLK_PIN);
    DL_GPIO_clearPins(GPIO_TFT_PORT, GPIO_TFT_CS_PIN);

    /* Hardware reset */
    DL_GPIO_clearPins(GPIO_TFT_PORT, GPIO_TFT_RES_PIN);
    delay_cycles(DELAY_MS(20));
    DL_GPIO_setPins(GPIO_TFT_PORT, GPIO_TFT_RES_PIN);
    delay_cycles(DELAY_MS(120));

    lcdCmd(0x01); /* software reset */
    delay_cycles(DELAY_MS(150));
    lcdCmd(0x11); /* sleep out */
    delay_cycles(DELAY_MS(120));

    lcdCmd(0xB1); /* frame rate control */
    lcdDat((const uint8_t[]){0x01, 0x2C, 0x2D}, 3);
    lcdCmd(0xB2);
    lcdDat((const uint8_t[]){0x01, 0x2C, 0x2D}, 3);
    lcdCmd(0xB3);
    lcdDat((const uint8_t[]){0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}, 6);
    lcdCmd(0xB4); /* inversion control */
    lcdDat((const uint8_t[]){0x07}, 1);

    lcdCmd(0xC0); /* power control */
    lcdDat((const uint8_t[]){0xA2, 0x02, 0x84}, 3);
    lcdCmd(0xC1);
    lcdDat((const uint8_t[]){0xC5}, 1);
    lcdCmd(0xC2);
    lcdDat((const uint8_t[]){0x0A, 0x00}, 2);
    lcdCmd(0xC3);
    lcdDat((const uint8_t[]){0x8A, 0x2A}, 2);
    lcdCmd(0xC4);
    lcdDat((const uint8_t[]){0x8A, 0xEE}, 2);
    lcdCmd(0xC5); /* VCOM */
    lcdDat((const uint8_t[]){0x0E}, 1);

    lcdCmd(0x20); /* inversion off */
    lcdCmd(0x36); /* data access control: mirrored + BGR */
    lcdDat((const uint8_t[]){0xC8}, 1);
    lcdCmd(0x3A); /* 16 bit colour */
    lcdDat((const uint8_t[]){0x05}, 1);
    lcdCmd(0x29); /* display on */
    delay_cycles(DELAY_MS(100));
}

#if LCD_TEST
/* Red, green, blue flashes followed by a colour bar pattern */
static void lcdTest(void)
{
    static const uint16_t bars[8] = {LCD_RGB565(255, 0, 0),
        LCD_RGB565(0, 255, 0), LCD_RGB565(0, 0, 255),
        LCD_RGB565(255, 255, 0), LCD_RGB565(0, 255, 255),
        LCD_RGB565(255, 0, 255), LCD_RGB565(255, 255, 255),
        LCD_RGB565(0, 0, 0)};

    lcdInit();

    lcdRect(0, 0, LCD_W - 1, LCD_H - 1, LCD_RGB565(255, 0, 0));
    delay_cycles(DELAY_MS(700));
    lcdRect(0, 0, LCD_W - 1, LCD_H - 1, LCD_RGB565(0, 255, 0));
    delay_cycles(DELAY_MS(700));
    lcdRect(0, 0, LCD_W - 1, LCD_H - 1, LCD_RGB565(0, 0, 255));
    delay_cycles(DELAY_MS(700));

    for (uint16_t i = 0; i < 8; i++) {
        lcdRect(i * 16, 0, i * 16 + 15, LCD_H - 1, bars[i]);
    }

    /* 1-pixel white frame: with the right offset all four edges are complete.
     * Garbage under the bottom line -> YOFF too small; the top line missing
     * -> YOFF too big. */
    lcdRect(0, 0, LCD_W - 1, 0, LCD_RGB565(255, 255, 255));
    lcdRect(0, LCD_H - 1, LCD_W - 1, LCD_H - 1, LCD_RGB565(255, 255, 255));
    lcdRect(0, 0, 0, LCD_H - 1, LCD_RGB565(255, 255, 255));
    lcdRect(LCD_W - 1, 0, LCD_W - 1, LCD_H - 1, LCD_RGB565(255, 255, 255));
}
#endif /* LCD_TEST */

#if IMU_TEST
/* -------------------------------- IMU963RA -------------------------------
 * SeekFree IMU963RA on I2C1: SDA = PA16, SCL = PA17. The six-axis part answers
 * at 7-bit address 0x68, the magnetometer (QMC5883L family) at 0x2C.
 * At power up the bus is scanned, the found devices are printed on the debug
 * UART and a square in the middle of the screen shows the result:
 *   green = both, yellow = only 0x68, blue = only 0x2C, red = nothing.
 */
#define IMU_ADDR_6AXIS (0x68)
#define IMU_ADDR_MAG   (0x2C)

/* Sends a one byte read and reports whether the address was acknowledged */
static bool imuPing(uint8_t addr)
{
    uint32_t guard = 0;

    while (!(DL_I2C_getControllerStatus(I2C_IMU_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (++guard > 2000000) {
            return false;
        }
    }

    DL_I2C_flushControllerRXFIFO(I2C_IMU_INST);
    DL_I2C_startControllerTransfer(
        I2C_IMU_INST, addr, DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    guard = 0;
    while (DL_I2C_getControllerStatus(I2C_IMU_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (++guard > 2000000) {
            return false;
        }
    }

    bool acked = (DL_I2C_getControllerStatus(I2C_IMU_INST) &
                  DL_I2C_CONTROLLER_STATUS_ADDR_ACK) != 0;

    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_IMU_INST)) {
        (void) DL_I2C_receiveControllerData(I2C_IMU_INST);
    }
    return acked;
}

/* Reads len bytes starting at register reg (write reg, repeated start, read) */
static bool imuRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t  r     = reg;
    uint32_t guard = 0;

    while (!(DL_I2C_getControllerStatus(I2C_IMU_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
    }
    DL_I2C_flushControllerRXFIFO(I2C_IMU_INST);

    DL_I2C_fillControllerTXFIFO(I2C_IMU_INST, &r, 1);
    DL_I2C_startControllerTransferAdvanced(I2C_IMU_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);

    while (DL_I2C_getControllerStatus(I2C_IMU_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (++guard > 2000000) {
            return false;
        }
    }

    DL_I2C_startControllerTransferAdvanced(I2C_IMU_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, (uint16_t) len,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    for (uint32_t i = 0; i < len; i++) {
        guard = 0;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_IMU_INST)) {
            if (++guard > 2000000) {
                return false;
            }
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_IMU_INST);
    }

    guard = 0;
    while (DL_I2C_getControllerStatus(I2C_IMU_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (++guard > 2000000) {
            return false;
        }
    }

    return (DL_I2C_getControllerStatus(I2C_IMU_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) == 0;
}

static void imuTest(void)
{
    uint8_t  id75 = 0, id72 = 0, id6b = 0, id6a = 0;
    bool     r75 = false, r72 = false, r6b = false, r6a = false;
    bool     found6  = imuPing(IMU_ADDR_6AXIS);
    bool     foundM  = imuPing(IMU_ADDR_MAG);
    bool     found6b = imuPing(0x6B);
    bool     found6a = imuPing(0x6A);

    dbgStr("\r\nI2C scan: 0x68 ");
    dbgStr(found6 ? "ACK" : "---");
    dbgStr(", 0x2C ");
    dbgStr(foundM ? "ACK" : "---");
    dbgStr("\r\n");

    if (found6) {
        r75 = imuRead(IMU_ADDR_6AXIS, 0x75, &id75, 1);
        r72 = imuRead(IMU_ADDR_6AXIS, 0x72, &id72, 1);
    }
    if (found6b) {
        r6b = imuRead(0x6B, 0x0F, &id6b, 1);
    }
    if (found6a) {
        r6a = imuRead(0x6A, 0x0F, &id6a, 1);
    }

    /* DIAG: show the scan on the LCD so a photo of the screen is enough.
     * 68 / 2C = the addresses the old code used, 6B / 6A = LSM6DSR.
     * 75 / 72 = WHO registers on the 0x68 chip, 0F = WHO on 6B / 6A. */
    lcdInit();
    lcdRect(0, 0, LCD_W - 1, LCD_H - 1, LCD_RGB565(0, 0, 0));
    lcdText(0, 0, "SCAN", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));

    lcdText(0, 16, "68", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    lcdText(32, 16, found6 ? "ACK" : "---",
        LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    lcdText(0, 32, "2C", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    lcdText(32, 32, foundM ? "ACK" : "---",
        LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));

    lcdText(0, 48, "75", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    if (r75) {
        lcdHex(32, 48, id75, 2, LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    } else {
        lcdText(32, 48, "--", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    }
    lcdText(0, 64, "72", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    if (r72) {
        lcdHex(32, 64, id72, 2, LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    } else {
        lcdText(32, 64, "--", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    }

    lcdText(0, 80, "6B", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    lcdText(32, 80, found6b ? "ACK" : "---",
        LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    if (r6b) {
        lcdHex(80, 80, id6b, 2, LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    }
    lcdText(0, 96, "6A", LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    lcdText(32, 96, found6a ? "ACK" : "---",
        LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    if (r6a) {
        lcdHex(80, 96, id6a, 2, LCD_RGB565(255, 255, 255), LCD_RGB565(0, 0, 0));
    }

    while (1) {
        /* hold the result on screen */
    }
}

#endif /* IMU_TEST */

#if (GYRO_TEST || FLUID_TEST)
/* ------------------------------ IMU963RA gyro ----------------------------
 * FIXED 2026-11-19: the six-axis part of the IMU963RA is an ST LSM6DSR,
 * NOT an InvenSense/MPU6050 part. The old code used the MPU6050 map
 * (address 0x68, WHO 0x75, data 0x3B, big-endian), so the sensor never
 * ACKed, every read failed and GX/GY/GZ were never drawn.
 * LSM6DSR map used below:
 *   address 0x6B (SDO high) or 0x6A (SDO low) - probed at start-up
 *   WHO_AM_I register 0x0F, expected value 0x6B
 *   gyro data from 0x22, accel data from 0x28, LOW byte first
 *   CTRL1_XL 0x10 / CTRL2_G 0x11 / CTRL3_C 0x12 must be written to start
 *   the sensors (power-on default is ODR = 0, i.e. powered down).
 * Init values match the SeekFree zf_device_imu963ra driver.
 */
#define GYRO_REG_WHO    (0x0F) /* WHO_AM_I */
#define GYRO_WHO_VALUE  (0x6B)
#define GYRO_DATA_G     (0x22) /* OUTX_L_G, 12 bytes: gyro then accel */

static uint8_t gyro_addr = 0x6B; /* probed address, 0x6B or 0x6A */
#define LCD_FG        LCD_RGB565(255, 255, 255)
#define LCD_BG        LCD_RGB565(0, 0, 0)

/* --------------------------- software I2C ---------------------------------
 * The hardware I2C peripheral would only acknowledge the address, every
 * register read failed on this module, so the bus is bit-banged on the same
 * two pins instead. Both lines carry pull-ups on the sensor module: "high"
 * releases the pin as an input, "low" drives it as an output.
 */
#define SI2C_SDA_PORT  GPIO_I2C_IMU_SDA_PORT
#define SI2C_SDA_PIN   GPIO_I2C_IMU_SDA_PIN
#define SI2C_SDA_IOMUX GPIO_I2C_IMU_IOMUX_SDA
#define SI2C_SCL_PORT  GPIO_I2C_IMU_SCL_PORT
#define SI2C_SCL_PIN   GPIO_I2C_IMU_SCL_PIN
#define SI2C_SCL_IOMUX GPIO_I2C_IMU_IOMUX_SCL
#define SI2C_HALF()    delay_cycles(160) /* ~5 us half period -> ~100 kHz */

static void si2cRelease(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(iomux, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void si2cLow(GPIO_Regs *port, uint32_t pin, uint32_t iomux)
{
    DL_GPIO_initDigitalOutput(iomux);
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

static void sdaHigh(void)
{
    /* Output must be disabled explicitly: switching the pin back to an input
     * does not clear the output enable, so a pin that was driven low once
     * would stay low forever (and make every address look ACKed). */
    DL_GPIO_disableOutput(SI2C_SDA_PORT, SI2C_SDA_PIN);
    si2cRelease(SI2C_SDA_IOMUX);
}

static void sdaLow(void)
{
    si2cLow(SI2C_SDA_PORT, SI2C_SDA_PIN, SI2C_SDA_IOMUX);
}

static void sclHigh(void)
{
    DL_GPIO_disableOutput(SI2C_SCL_PORT, SI2C_SCL_PIN);
    si2cRelease(SI2C_SCL_IOMUX);
}

static void sclLow(void)
{
    si2cLow(SI2C_SCL_PORT, SI2C_SCL_PIN, SI2C_SCL_IOMUX);
}

static bool sdaRead(void)
{
    return DL_GPIO_readPins(SI2C_SDA_PORT, SI2C_SDA_PIN) != 0;
}

static void si2cStart(void)
{
    sdaHigh();
    sclHigh();
    SI2C_HALF();
    sdaLow();
    SI2C_HALF();
    sclLow();
    SI2C_HALF();
}

static void si2cStop(void)
{
    sdaLow();
    SI2C_HALF();
    sclHigh();
    SI2C_HALF();
    sdaHigh();
    SI2C_HALF();
}

static bool si2cWriteByte(uint8_t b)
{
    bool ack;

    for (int32_t i = 7; i >= 0; i--) {
        if (b & (1u << i)) {
            sdaHigh();
        } else {
            sdaLow();
        }
        SI2C_HALF();
        sclHigh();
        SI2C_HALF();
        sclLow();
        SI2C_HALF();
    }

    sdaHigh(); /* release SDA for the acknowledge bit */
    SI2C_HALF();
    sclHigh();
    SI2C_HALF();
    ack = !sdaRead();
    sclLow();
    SI2C_HALF();

    return ack;
}

static uint8_t si2cReadByte(bool ack)
{
    uint8_t b = 0;

    sdaHigh(); /* the sensor drives SDA now */

    for (int32_t i = 7; i >= 0; i--) {
        SI2C_HALF();
        sclHigh();
        SI2C_HALF();
        b = (uint8_t)((b << 1) | (sdaRead() ? 1 : 0));
        sclLow();
        SI2C_HALF();
    }

    if (ack) {
        sdaLow();
    } else {
        sdaHigh();
    }
    SI2C_HALF();
    sclHigh();
    SI2C_HALF();
    sclLow();
    SI2C_HALF();
    sdaHigh();

    return b;
}

static bool si2cWriteReg(uint8_t reg, uint8_t val)
{
    bool ok;

    si2cStart();
    ok  = si2cWriteByte((uint8_t)(gyro_addr << 1));
    ok &= si2cWriteByte(reg);
    ok &= si2cWriteByte(val);
    si2cStop();

    return ok;
}

static bool si2cReadReg(uint8_t reg, uint8_t *buf, uint32_t len)
{
    bool ok;

    si2cStart();
    ok = si2cWriteByte((uint8_t)(gyro_addr << 1));
    if (ok) {
        ok = si2cWriteByte(reg);
    }
    if (ok) {
        si2cStart(); /* repeated start */
        ok = si2cWriteByte((uint8_t)((gyro_addr << 1) | 1));
    }
    if (!ok) {
        si2cStop();
        return false;
    }

    for (uint32_t i = 0; i < len; i++) {
        buf[i] = si2cReadByte(i + 1 < len);
    }
    si2cStop();

    return true;
}

static bool gyroReadData(int16_t *acc, int16_t *gyr)
{
    uint8_t b[12];

    if (!si2cReadReg(GYRO_DATA_G, b, 12)) {
        return false;
    }

    /* LSM6DSR: low byte first (the MPU6050 order was the exact opposite) */
    gyr[0] = (int16_t)(((uint16_t) b[1] << 8) | b[0]);
    gyr[1] = (int16_t)(((uint16_t) b[3] << 8) | b[2]);
    gyr[2] = (int16_t)(((uint16_t) b[5] << 8) | b[4]);
    acc[0] = (int16_t)(((uint16_t) b[7] << 8) | b[6]);
    acc[1] = (int16_t)(((uint16_t) b[9] << 8) | b[8]);
    acc[2] = (int16_t)(((uint16_t) b[11] << 8) | b[10]);
    return true;
}

/* Gyro readout: never returns */
/* Average the gyro over about a second while the board sits still. The
 * result is subtracted from every later sample, which removes the fixed
 * zero-rate offset (that offset would otherwise integrate into tens of
 * degrees per minute). */
static void gyroCalibrate(int32_t *bias, int32_t *accRef)
{
    int32_t sum[3]  = {0, 0, 0};
    int32_t asum[3] = {0, 0, 0};
    int32_t used    = 0;

    for (int32_t i = 0; i < 200; i++) {
        int16_t acc[3];
        int16_t gyr[3];

        if (gyroReadData(acc, gyr)) {
            sum[0] += gyr[0];
            sum[1] += gyr[1];
            sum[2] += gyr[2];
            asum[0] += acc[0];
            asum[1] += acc[1];
            asum[2] += acc[2];
            used++;
        }
        delay_cycles(DELAY_MS(10));
    }

    if (used == 0) {
        used = 1;
    }
    for (int32_t i = 0; i < 3; i++) {
        /* round to nearest instead of truncating */
        int32_t s = sum[i];

        bias[i]   = (s >= 0) ? (s + used / 2) / used : (s - used / 2) / used;
        accRef[i] = asum[i] / used;
    }
}

/* Gravity vector -> roll and pitch, in tenths of a degree */
static void accelAngles(
    int32_t axC, int32_t ayC, int32_t azC, int32_t *roll10, int32_t *pitch10)
{
    float ax    = (float) axC * 0.000244f;
    float ay    = (float) ayC * 0.000244f;
    float az    = (float) azC * 0.000244f;
    float horiz = 0.0f;
    float roll  = 0.0f;
    float pitch = 0.0f;

    (void) arm_sqrt_f32(ay * ay + az * az, &horiz);
    (void) arm_atan2_f32(ay, az, &roll);
    (void) arm_atan2_f32(-ax, horiz, &pitch);
    *roll10  = (int32_t)(roll * 572.9578f);
    *pitch10 = (int32_t)(pitch * 572.9578f);
}

/* Probes 0x6B and 0x6A, resets the LSM6DSR and writes its init sequence.
 * Returns true and the WHO_AM_I value when the expected sensor answered. */
static bool imuInit(uint8_t *whoOut)
{
    const uint8_t cand[2] = {0x6B, 0x6A};
    uint8_t       who     = 0;
    bool          ok      = false;

    for (uint32_t a = 0; a < 2 && !ok; a++) {
        gyro_addr = cand[a];
        (void) si2cWriteReg(0x01, 0x00);
        (void) si2cWriteReg(0x12, 0x01); /* CTRL3_C: software reset */
        delay_cycles(DELAY_MS(10));
        for (uint32_t t = 0; t < 20 && !ok; t++) {
            if (si2cReadReg(GYRO_REG_WHO, &who, 1) &&
                who == GYRO_WHO_VALUE) {
                ok = true;
            } else {
                delay_cycles(DELAY_MS(10));
            }
        }
    }

    if (ok) {
        /* LSM6DSR init (values from the SeekFree driver) */
        si2cWriteReg(0x0D, 0x03); /* INT1_CTRL */
        si2cWriteReg(0x10, 0x3C); /* CTRL1_XL: accel +-8 g, ODR on */
        si2cWriteReg(0x11, 0x5C); /* CTRL2_G: gyro +-2000 dps, ODR on */
        si2cWriteReg(0x12, 0x44); /* CTRL3_C: BDU + register auto-increment */
        si2cWriteReg(0x13, 0x02); /* CTRL4_C */
        si2cWriteReg(0x14, 0x00); /* CTRL5_C */
        si2cWriteReg(0x15, 0x00); /* CTRL6_C */
        si2cWriteReg(0x16, 0x00); /* CTRL7_G */
        si2cWriteReg(0x18, 0x01); /* CTRL9_XL: I3C interface off */
        delay_cycles(DELAY_MS(200)); /* settle before calibration */
    }

    *whoOut = who;
    return ok;
}

#if GYRO_TEST
static void gyroRun(void)
{
    int16_t acc[3]    = {0, 0, 0};
    int16_t gyr[3]    = {0, 0, 0};
    int32_t accF[3]   = {0, 0, 0}; /* filtered accel, Q8.8 counts */
    int32_t accRef[3] = {0, 0, 0}; /* gravity vector at power up */
    int32_t bias[3]   = {0, 0, 0};
    int32_t yawU      = 0; /* half-millidegrees */
    int32_t roll0     = 0; /* reference attitude, tenths of a degree */
    int32_t pitch0    = 0;
    bool    accFInit  = false;
    uint8_t who       = 0;
    bool    okWho     = false;

    lcdInit();
    lcdRect(0, 0, LCD_W - 1, LCD_H - 1, LCD_BG);

    okWho = imuInit(&who);

    lcdText(0, 0, "WHO", LCD_FG, LCD_BG);
    if (okWho) {
        lcdHex(32, 0, who, 2, LCD_FG, LCD_BG);
    } else {
        lcdText(32, 0, "--", LCD_FG, LCD_BG);
    }

    if (okWho) {
        lcdText(0, 112, "CAL", LCD_FG, LCD_BG); /* keep the board still */
        gyroCalibrate(bias, accRef);
        accelAngles(accRef[0], accRef[1], accRef[2], &roll0, &pitch0);
        lcdText(0, 112, "   ", LCD_FG, LCD_BG);
    }

    while (1) {
        /* Pressing a button re-calibrates and zeroes the yaw */
        if (startStopPressed()) {
            yawU = 0;
            lcdText(0, 112, "CAL", LCD_FG, LCD_BG);
            gyroCalibrate(bias, accRef);
            accelAngles(accRef[0], accRef[1], accRef[2], &roll0, &pitch0);
            lcdText(0, 112, "   ", LCD_FG, LCD_BG);
        }

        bool ok = gyroReadData(acc, gyr);

        if (ok) {
            int32_t gz      = (int32_t) gyr[2] - bias[2];
            int32_t mg[3];
            int32_t roll10  = 0;
            int32_t pitch10 = 0;

            /* Yaw integration at one sample per ~50 ms: one LSB is 70 mdps,
             * so one sample adds 70 * 0.05 = 3.5 mdeg = 7 half-mdeg. */
            yawU += gz * 7;

            /* 8-tap exponential low pass on the accelerometer (time constant
             * about 0.4 s). Tilt is slow, so the noise can be filtered away. */
            if (!accFInit) {
                accF[0]  = (int32_t) acc[0] << 8;
                accF[1]  = (int32_t) acc[1] << 8;
                accF[2]  = (int32_t) acc[2] << 8;
                accFInit = true;
            } else {
                for (int32_t i = 0; i < 3; i++) {
                    accF[i] += (((int32_t) acc[i] << 8) - accF[i]) >> 3;
                }
            }

            /* Accel: 0.244 mg/LSB at +-8 g, shown in g with three decimals */
            mg[0] = (int32_t)(accF[0] >> 8) * 244 / 1000;
            mg[1] = (int32_t)(accF[1] >> 8) * 244 / 1000;
            mg[2] = (int32_t)(accF[2] >> 8) * 244 / 1000;

            /* Attitude relative to the power-up orientation */
            accelAngles(accF[0] >> 8, accF[1] >> 8, accF[2] >> 8, &roll10,
                &pitch10);
            roll10  -= roll0;
            pitch10 -= pitch0;

            /* Predicted 2-D gravity for the FLIP sim: same Q8.8 scale as
             * F_GRAVITY (24). gx = G*sin(roll), gy = G*cos(roll), with the
             * tilt clamped to +-45 degrees. */
            int32_t tilt = roll10;
            int32_t gsx;
            int32_t gsy;

            if (tilt > 450) {
                tilt = 450;
            } else if (tilt < -450) {
                tilt = -450;
            }
            gsx = tilt * 419 / 10000;
            gsy = 24 - (gsx * gsx) / 48;

            lcdText(0, 0, "YAW", LCD_FG, LCD_BG);
            lcdInt(32, 0, yawU / 2000, 6, LCD_FG, LCD_BG);
            lcdText(0, 14, "AX", LCD_FG, LCD_BG);
            lcdFixed(24, 14, mg[0], 3, 6, LCD_FG, LCD_BG);
            lcdText(0, 28, "AY", LCD_FG, LCD_BG);
            lcdFixed(24, 28, mg[1], 3, 6, LCD_FG, LCD_BG);
            lcdText(0, 42, "AZ", LCD_FG, LCD_BG);
            lcdFixed(24, 42, mg[2], 3, 6, LCD_FG, LCD_BG);
            lcdText(0, 56, "ROL", LCD_FG, LCD_BG);
            lcdFixed(32, 56, roll10, 1, 6, LCD_FG, LCD_BG);
            lcdText(0, 70, "PIT", LCD_FG, LCD_BG);
            lcdFixed(32, 70, pitch10, 1, 6, LCD_FG, LCD_BG);
            lcdText(0, 84, "GSX", LCD_FG, LCD_BG);
            lcdInt(32, 84, gsx, 6, LCD_FG, LCD_BG);
            lcdText(0, 98, "GSY", LCD_FG, LCD_BG);
            lcdInt(32, 98, gsy, 6, LCD_FG, LCD_BG);
        }

        lcdText(0, 112, ok ? "    " : "ERR", LCD_FG, LCD_BG);
        delay_cycles(DELAY_MS(50));
    }
}
#endif /* GYRO_TEST */

#endif /* GYRO_TEST || FLUID_TEST */

#if FLUID_TEST
/* -------------------------------- FLIP water -----------------------------
 * 2D FLIP/PIC water on the 128x128 panel. Simulation grid 32x32 cells, one
 * cell is 4x4 pixels, 300 particles. Everything is Q8.8 fixed point (1.0 =
 * 256) because the M0+ has no FPU. Particle velocities are in cells per
 * frame, so the advection step is simply x += vx.
 *
 * Per frame: sample the old grid velocity (needed for the FLIP part),
 * apply gravity, P2G transfer, pressure projection (Gauss-Seidel), G2P with
 * the FLIP blend, advect, then repaint only the cells that changed.
 */
#define F_N             (32)
#define F_NP            (600)
#define F_GRAVITY       (24)   /* cells / frame^2 */
#define F_FLIP_KEEP     (230)  /* Q8.8, 0.9: keep 90 % of the particle velocity */
#define F_PRESS_ITERS   (30)
#define F_OMEGA         (486)  /* Q8.8, 1.9: SOR over-relaxation factor */
#define F_VMAX          (1024) /* clamp: 4 cells per frame */
#define F_WATER         LCD_RGB565(0, 120, 255)
#define F_BG            LCD_RGB565(0, 0, 0)

static int16_t fu[F_N + 1][F_N];     /* x velocity, vertical faces */
static int16_t fv[F_N][F_N + 1];     /* y velocity, horizontal faces */
static int16_t fuw[F_N + 1][F_N];    /* weight sums of the P2G transfer */
static int16_t fvw[F_N][F_N + 1];
static int16_t fp[F_N][F_N];         /* pressure */
static int16_t fd[F_N][F_N];         /* divergence */
static int16_t fpx[F_NP], fpy[F_NP]; /* particle position, cells (Q8.8) */
static int16_t fvx[F_NP], fvy[F_NP]; /* particle velocity, cells/frame */
static int16_t fox[F_NP], foy[F_NP]; /* grid velocity sampled before P2G */
static uint8_t fcount[F_N][F_N];
static uint8_t fshown[F_N][F_N];
static int32_t fTopJ; /* first row that holds particles, minus a margin */
static int16_t fGx;   /* gravity across the screen, set from the IMU roll */
static int16_t fGy = F_GRAVITY; /* gravity down the screen */

static int32_t fclamp(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static int32_t fmin32(int32_t a, int32_t b)
{
    return (a < b) ? a : b;
}

/* Round-to-nearest, symmetric shift. A plain >> floors, so every negative
 * product loses up to one LSB. Applied per particle per frame that tiny bias
 * behaves like a constant acceleration: over a few hundred frames the water
 * slowly slides to the left wall (and a tilt in that direction blows up). */
static int32_t frshr(int32_t v, int32_t s)
{
    if (v >= 0) {
        return (v + (1 << (s - 1))) >> s;
    }
    return -((-v + (1 << (s - 1))) >> s);
}

/* Bilinear sample of the u faces (u sits at x = i, y = j + 0.5) */
static int32_t fsampleU(int32_t x, int32_t y)
{
    int32_t i  = fclamp(x >> 8, 0, F_N - 1);
    int32_t j  = fclamp((y - 128) >> 8, 0, F_N - 1);
    int32_t j1 = fmin32(j + 1, F_N - 1);
    int32_t fx = x & 0xFF;
    int32_t fy = (y - 128) & 0xFF;
    int32_t a  = fu[i][j];
    int32_t b  = fu[i + 1][j];
    int32_t c  = fu[i][j1];
    int32_t d  = fu[i + 1][j1];
    int32_t top = a + frshr((b - a) * fx, 8);
    int32_t bot = c + frshr((d - c) * fx, 8);

    return top + frshr((bot - top) * fy, 8);
}

/* Bilinear sample of the v faces (v sits at x = i + 0.5, y = j) */
static int32_t fsampleV(int32_t x, int32_t y)
{
    int32_t i  = fclamp((x - 128) >> 8, 0, F_N - 1);
    int32_t i1 = fmin32(i + 1, F_N - 1);
    int32_t j  = fclamp(y >> 8, 0, F_N - 1);
    int32_t fx = (x - 128) & 0xFF;
    int32_t fy = y & 0xFF;
    int32_t a  = fv[i][j];
    int32_t b  = fv[i][j + 1];
    int32_t c  = fv[i1][j];
    int32_t d  = fv[i1][j + 1];
    int32_t top = a + frshr((c - a) * fx, 8);
    int32_t bot = b + frshr((d - b) * fx, 8);

    return top + frshr((bot - top) * fy, 8);
}

static void fclearGrids(void)
{
    for (int32_t i = 0; i <= F_N; i++) {
        for (int32_t j = 0; j < F_N; j++) {
            fu[i][j]  = 0;
            fuw[i][j] = 0;
        }
    }
    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = 0; j <= F_N; j++) {
            fv[i][j]  = 0;
            fvw[i][j] = 0;
        }
    }
}

/* Particle -> grid transfer (bilinear weights) */
static void fp2g(void)
{
    for (int32_t p = 0; p < F_NP; p++) {
        int32_t x  = fpx[p];
        int32_t y  = fpy[p];
        int32_t vx = fvx[p];
        int32_t vy = fvy[p];
        int32_t i, i1, j, j1, fx, fy, w00, w10, w01, w11;

        /* u faces */
        i  = fclamp(x >> 8, 0, F_N - 1);
        j  = fclamp((y - 128) >> 8, 0, F_N - 1);
        j1 = fmin32(j + 1, F_N - 1);
        fx  = x & 0xFF;
        fy  = (y - 128) & 0xFF;
        w00 = ((256 - fx) * (256 - fy)) >> 8;
        w10 = (fx * (256 - fy)) >> 8;
        w01 = ((256 - fx) * fy) >> 8;
        w11 = (fx * fy) >> 8;

        fu[i][j] = (int16_t) fclamp(
            fu[i][j] + frshr(vx * w00, 8), -30000, 30000);
        fu[i + 1][j] = (int16_t) fclamp(
            fu[i + 1][j] + frshr(vx * w10, 8), -30000, 30000);
        fu[i][j1] = (int16_t) fclamp(
            fu[i][j1] + frshr(vx * w01, 8), -30000, 30000);
        fu[i + 1][j1] = (int16_t) fclamp(
            fu[i + 1][j1] + frshr(vx * w11, 8), -30000, 30000);

        fuw[i][j]     = (int16_t) fclamp(fuw[i][j] + w00, 0, 30000);
        fuw[i + 1][j] = (int16_t) fclamp(fuw[i + 1][j] + w10, 0, 30000);
        fuw[i][j1]    = (int16_t) fclamp(fuw[i][j1] + w01, 0, 30000);
        fuw[i + 1][j1] = (int16_t) fclamp(fuw[i + 1][j1] + w11, 0, 30000);

        /* v faces */
        i  = fclamp((x - 128) >> 8, 0, F_N - 1);
        i1 = fmin32(i + 1, F_N - 1);
        j  = fclamp(y >> 8, 0, F_N - 1);
        fx = (x - 128) & 0xFF;
        fy = y & 0xFF;
        w00 = ((256 - fx) * (256 - fy)) >> 8;
        w10 = (fx * (256 - fy)) >> 8;
        w01 = ((256 - fx) * fy) >> 8;
        w11 = (fx * fy) >> 8;

        fv[i][j] = (int16_t) fclamp(
            fv[i][j] + frshr(vy * w00, 8), -30000, 30000);
        fv[i1][j] = (int16_t) fclamp(
            fv[i1][j] + frshr(vy * w10, 8), -30000, 30000);
        fv[i][j + 1] = (int16_t) fclamp(
            fv[i][j + 1] + frshr(vy * w01, 8), -30000, 30000);
        fv[i1][j + 1] = (int16_t) fclamp(
            fv[i1][j + 1] + frshr(vy * w11, 8), -30000, 30000);

        fvw[i][j]         = (int16_t) fclamp(fvw[i][j] + w00, 0, 30000);
        fvw[i1][j]        = (int16_t) fclamp(fvw[i1][j] + w10, 0, 30000);
        fvw[i][j + 1]     = (int16_t) fclamp(fvw[i][j + 1] + w01, 0, 30000);
        fvw[i1][j + 1]    = (int16_t) fclamp(fvw[i1][j + 1] + w11, 0, 30000);
    }
}

/* Divide the accumulated sums by the weights and zero the wall faces */
static void fnormalise(void)
{
    for (int32_t i = 0; i <= F_N; i++) {
        for (int32_t j = 0; j < F_N; j++) {
            int32_t w = fuw[i][j];

            if (w > 8) {
                fu[i][j] = (int16_t) fclamp(
                    (((int32_t) fu[i][j] * 256) / w), -F_VMAX, F_VMAX);
            } else {
                fu[i][j] = 0;
            }
        }
    }
    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = 0; j <= F_N; j++) {
            int32_t w = fvw[i][j];

            if (w > 8) {
                fv[i][j] = (int16_t) fclamp(
                    (((int32_t) fv[i][j] * 256) / w), -F_VMAX, F_VMAX);
            } else {
                fv[i][j] = 0;
            }
        }
    }

    /* solid walls: no flow through the outer faces */
    for (int32_t j = 0; j < F_N; j++) {
        fu[0][j] = 0;
        fu[F_N][j] = 0;
    }
    for (int32_t i = 0; i < F_N; i++) {
        fv[i][0] = 0;
        fv[i][F_N] = 0;
    }
}

static void fdivergence(void)
{
    /* only the rows that contain fluid matter - the rest is empty air */
    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = fTopJ; j < F_N; j++) {
            fd[i][j] = (int16_t)(fu[i + 1][j] - fu[i][j] + fv[i][j + 1] -
                                 fv[i][j]);
        }
    }
}

static void fpressure(void)
{
    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = 0; j < F_N; j++) {
            fp[i][j] = 0;
        }
    }

    /* Gauss-Seidel with SOR, mirrors at the walls and at the top of the
     * fluid band (Neumann boundaries) */
    for (int32_t k = 0; k < F_PRESS_ITERS; k++) {
        for (int32_t i = 0; i < F_N; i++) {
            for (int32_t j = fTopJ; j < F_N; j++) {
                int32_t pL = (i > 0) ? fp[i - 1][j] : fp[i][j];
                int32_t pR = (i < F_N - 1) ? fp[i + 1][j] : fp[i][j];
                int32_t pU = (j > fTopJ) ? fp[i][j - 1] : fp[i][j];
                int32_t pD = (j < F_N - 1) ? fp[i][j + 1] : fp[i][j];
                int32_t gs = frshr(pL + pR + pU + pD - fd[i][j], 2);
                int32_t v  = fp[i][j] + frshr((gs - fp[i][j]) * F_OMEGA, 8);

                fp[i][j] = (int16_t) fclamp(v, -16000, 16000);
            }
        }
    }

    /* subtract the pressure gradient from the face velocities */
    for (int32_t i = 1; i < F_N; i++) {
        for (int32_t j = fTopJ; j < F_N; j++) {
            fu[i][j] = (int16_t) fclamp(
                fu[i][j] - (fp[i][j] - fp[i - 1][j]), -F_VMAX, F_VMAX);
        }
    }
    for (int32_t j = fTopJ + 1; j < F_N; j++) {
        for (int32_t i = 0; i < F_N; i++) {
            fv[i][j] = (int16_t) fclamp(
                fv[i][j] - (fp[i][j] - fp[i][j - 1]), -F_VMAX, F_VMAX);
        }
    }
}

static void fluidStep(void)
{
    int32_t minRow = F_N - 1;

    /* band optimisation: only the rows that hold particles (plus two rows of
     * margin) take part in gravity, divergence and the pressure solve, so the
     * empty upper half of the screen costs nothing */
    for (int32_t p = 0; p < F_NP; p++) {
        int32_t r = fpy[p] >> 8;

        if (r < minRow) {
            minRow = r;
        }
    }
    fTopJ = fclamp(minRow - 2, 0, F_N - 1);

    /* remember the old grid velocity at every particle (FLIP term) */
    for (int32_t p = 0; p < F_NP; p++) {
        fox[p] = (int16_t) fsampleU(fpx[p], fpy[p]);
        foy[p] = (int16_t) fsampleV(fpx[p], fpy[p]);
    }

    fclearGrids();
    fp2g();
    fnormalise();

    /* Gravity goes on the grid, not on the particles: only then can the
     * projection cancel it for resting fluid (hydrostatic pressure).
     * Adding it to the particles and keeping it in the FLIP term makes the
     * whole pool sink into the floor.
     * fGy / fGx come from the IMU roll, so tilting the board tilts gravity
     * and the water runs "downhill". Walls are faces 0 and F_N, which are
     * never touched here (and were zeroed by fnormalise). */
    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = fTopJ + 1; j < F_N; j++) {
            fv[i][j] = (int16_t) fclamp(
                fv[i][j] + fGy, -F_VMAX, F_VMAX);
        }
    }
    if (fGx != 0) {
        for (int32_t i = 1; i < F_N; i++) {
            for (int32_t j = fTopJ; j < F_N; j++) {
                fu[i][j] = (int16_t) fclamp(
                    fu[i][j] + fGx, -F_VMAX, F_VMAX);
            }
        }
    }

    fdivergence();
    fpressure();

    /* back to the particles: FLIP blend, then advect and collide with walls */
    for (int32_t p = 0; p < F_NP; p++) {
        int32_t vgx = fsampleU(fpx[p], fpy[p]);
        int32_t vgy = fsampleV(fpx[p], fpy[p]);
        int32_t nvx =
            vgx + frshr((int32_t)(fvx[p] - fox[p]) * F_FLIP_KEEP, 8);
        int32_t nvy =
            vgy + frshr((int32_t)(fvy[p] - foy[p]) * F_FLIP_KEEP, 8);
        int32_t nx, ny;

        nvx = fclamp(nvx, -F_VMAX, F_VMAX);
        nvy = fclamp(nvy, -F_VMAX, F_VMAX);

        nx = fpx[p] + nvx;
        ny = fpy[p] + nvy;

        if (nx < 128) {
            nx  = 128;
            nvx = 0;
        }
        if (nx > F_N * 256 - 128) {
            nx  = F_N * 256 - 128;
            nvx = 0;
        }
        if (ny < 128) {
            ny  = 128;
            nvy = 0;
        }
        if (ny > F_N * 256 - 128) {
            ny  = F_N * 256 - 128;
            nvy = 0;
        }

        fpx[p] = (int16_t) nx;
        fpy[p] = (int16_t) ny;
        fvx[p] = (int16_t) nvx;
        fvy[p] = (int16_t) nvy;
    }
}

static void fluidInit(void)
{
    int32_t n = 0;

    /* a block of water in the bottom third: 30 x 10 cells, two particles
     * per cell at a quarter-cell offset, so the density starts uniform */
    for (int32_t j = 0; j < 10 && n < F_NP; j++) {
        for (int32_t i = 0; i < 30 && n < F_NP; i++) {
            for (int32_t k = 0; k < 2 && n < F_NP; k++) {
                int32_t jx = (k & 1) ? 64 : -64;

                fpx[n] = (int16_t)((i << 8) + 128 + jx);
                fpy[n] = (int16_t)(((F_N - 1 - j) << 8) + 128 - 64);
                fvx[n] = 0;
                fvy[n] = 0;
                n++;
            }
        }
    }

    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = 0; j < F_N; j++) {
            fp[i][j]     = 0;
            fd[i][j]     = 0;
            fcount[i][j] = 0;
            fshown[i][j] = 0;
        }
    }
    fclearGrids();
}

/* Count the particles per cell and repaint only the cells that changed */
static void fluidRender(void)
{
    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = 0; j < F_N; j++) {
            fcount[i][j] = 0;
        }
    }
    for (int32_t p = 0; p < F_NP; p++) {
        int32_t i = fclamp(fpx[p] >> 8, 0, F_N - 1);
        int32_t j = fclamp(fpy[p] >> 8, 0, F_N - 1);

        fcount[i][j]++;
    }

    for (int32_t i = 0; i < F_N; i++) {
        for (int32_t j = 0; j < F_N; j++) {
            uint8_t on = (fcount[i][j] != 0) ? 1 : 0;

            if (on != fshown[i][j]) {
                lcdRect((uint16_t)(i * 4), (uint16_t)(j * 4),
                    (uint16_t)(i * 4 + 3), (uint16_t)(j * 4 + 3),
                    on ? F_WATER : F_BG);
                fshown[i][j] = on;
            }
        }
    }
}

/* Entry point of the demo: never returns */
static void fluidRun(void)
{
    int32_t stir      = 1;
    int16_t acc[3]    = {0, 0, 0};
    int16_t gyr[3]    = {0, 0, 0};
    int32_t accF[3]   = {0, 0, 0}; /* filtered accel, Q8.8 counts */
    int32_t bias[3]   = {0, 0, 0};
    int32_t accRef[3] = {0, 0, 0}; /* gravity vector at power up */
    int32_t roll0     = 0; /* attitude when the demo started = "level" */
    int32_t pitch0    = 0;
    bool    accFInit  = false;
    uint8_t who       = 0;

    lcdInit();
    lcdRect(0, 0, LCD_W - 1, LCD_H - 1, F_BG);
    fluidInit();

    /* The IMU drives the gravity vector. Hold the board still once after
     * power-up in the position that should mean "level": that attitude is
     * stored as the zero point. Without the IMU the water simply falls
     * straight down, as before. */
    if (imuInit(&who)) {
        lcdText(0, 0, "CAL", LCD_FG, LCD_BG);
        gyroCalibrate(bias, accRef);
        accelAngles(accRef[0], accRef[1], accRef[2], &roll0, &pitch0);
        lcdRect(0, 0, 31, 7, F_BG); /* erase "CAL" */
    } else {
        lcdText(0, 0, "NO IMU", LCD_RGB565(255, 0, 0), F_BG);
    }

    while (1) {
        /* Roll -> horizontal gravity, once per frame (the I2C read is a few
         * ms). The water must run towards the side that is LOWER; the test on
         * the board showed right side down -> ROLL negative, so flip the sign:
         *   gx = -(G * sin(roll)), gy = G * cos(roll)
         * with 419/10000 ~ 24*pi/1800 for small angles and a +-45 deg clamp. */
        if (gyroReadData(acc, gyr)) {
            int32_t roll10  = 0;
            int32_t pitch10 = 0;
            int32_t rel;

            if (!accFInit) {
                accF[0]  = (int32_t) acc[0] << 8;
                accF[1]  = (int32_t) acc[1] << 8;
                accF[2]  = (int32_t) acc[2] << 8;
                accFInit = true;
            } else {
                for (int32_t i = 0; i < 3; i++) {
                    accF[i] += (((int32_t) acc[i] << 8) - accF[i]) >> 3;
                }
            }

            accelAngles(
                accF[0] >> 8, accF[1] >> 8, accF[2] >> 8, &roll10, &pitch10);

            rel = roll10 - roll0;
            if (rel > -4 && rel < 4) {
                rel = 0; /* deadband: sensor noise must not push the water */
            }
            rel = fclamp(rel, -450, 450);

            fGx = (int16_t)(-(rel * 419 / 10000));
            fGy = (int16_t)(F_GRAVITY - ((int32_t) fGx * fGx) / 48);
        }

        if (startStopPressed()) {
            /* stir the water sideways, alternating direction on every press */
            for (int32_t p = 0; p < F_NP; p++) {
                fvx[p] = (int16_t) fclamp(
                    fvx[p] + 256 * stir, -F_VMAX, F_VMAX);
            }
            stir = -stir;
        }

        fluidStep();
        fluidRender();
    }
}
#endif /* FLUID_TEST */

#endif /* LCD_ENABLED */

int main(void)
{
    bool running = false;

    /* Powers up the GPIO / UART / SPI peripherals used below */
    SYSCFG_DL_init();

#if GYRO_TEST
    gyroRun(); /* IMU963RA gyro readout: never returns */
#endif

#if FLUID_TEST
    fluidRun(); /* FLIP water demo: never returns */
#endif

#if LCD_TEST
    lcdTest(); /* red/green/blue flashes, then colour bars */
#endif

#if IMU_TEST
    imuTest(); /* scan the I2C bus, result square in the middle of the screen */
#endif

    dbgStr("\r\nMSPM0G3507 -> Emm42 closed loop stepper\r\n");
    dbgStr("driver address = ");
    dbgHex(EMM_ADDR);
    dbgStr("\r\npress a button to start / stop\r\n");

#if SERVO_TEST
    /* Continuous rotation servo: hold the stop pulse for a moment, then run
     * at full speed for half a turn and stop again. There is no holding
     * torque at 1.5 ms - that is normal for this type of servo. */
    DL_TimerG_startCounter(PWM_SERVO_INST);
    servoSetPulse(SERVO_STOP_US);
    delay_cycles(DELAY_MS(1000));

    servoSetPulse(SERVO_DRIVE_US);
    delay_cycles(DELAY_MS(SERVO_RUN_MS));
    servoSetPulse(SERVO_STOP_US);
    dbgStr("servo: stopped\r\n");
#endif

    /* Energise once at power up, so the first press moves the motor right
     * away. While stopped it still holds position (holding torque). */
    emmEnable(true);
    delay_cycles(DELAY_MS(10));

#if EMM_BOOT_TEST
    delay_cycles(DELAY_MS(2000));
    dbgStr("boot test: 5 s slow turn\r\n");
    emmSpeed(EMM_DIR_CW, EMM_RPM_LOW, EMM_ACCEL);
    delay_cycles(DELAY_MS(5000)); /* 5 s at 10 RPM -> about 0.8 turn */
    emmStop();
    dbgStr("boot test: done\r\n");
    delay_cycles(DELAY_MS(200));
#endif

    while (1) {
        if (startStopPressed()) {
            running = !running;

            if (running) {
                dbgStr("start\r\n");
                emmSpeed(EMM_DIR_CW, EMM_RPM_LOW, EMM_ACCEL);
            } else {
                dbgStr("stop\r\n");
                emmStop();
            }
        }

        delay_cycles(DELAY_MS(5));
    }
}
