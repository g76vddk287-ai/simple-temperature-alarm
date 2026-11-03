#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*
 * 功能：STM32 + MLX90614 + OLED + K210 串口联动
 * PB0：报警阈值 +0.5℃
 * PB1：报警阈值 -0.5℃
 * 注意：按键调整的是“报警阈值”，不是传感器测量的目标温度。
 */

#define I2C_GPIO        GPIOB
#define I2C_SCL_PIN     GPIO_Pin_6
#define I2C_SDA_PIN     GPIO_Pin_7
#define BUZZER_GPIO      GPIOB
#define BUZZER_PIN       GPIO_Pin_5
#define ALARM_TEMP_DEFAULT_10  350  /* 默认报警阈值：350 = 35.0℃ */
#define ALARM_TEMP_MIN_10      300  /* 最低报警阈值：300 = 30.0℃ */
#define ALARM_TEMP_MAX_10      450  /* 最高报警阈值：450 = 45.0℃ */
static int g_alarm_temp_10 = ALARM_TEMP_DEFAULT_10;  /* 当前报警阈值，单位0.1℃ */
#define MLX90614_ADDR   (0x5A << 1)
#define OLED_ADDR_3C    (0x3C << 1)
#define OLED_ADDR_3D    (0x3D << 1)
#define MLX_ADDR_00     (0x00 << 1)
#define MLX_ADDR_5A     (0x5A << 1)
#define MLX_ADDR_5B     (0x5B << 1)
static uint8_t g_oled_addr = OLED_ADDR_3C;
static uint8_t g_mlx_addr = MLX_ADDR_5A;
static uint8_t g_oled_ok = 0;
static uint8_t g_mlx_ok = 0;

#define SCL_H()         GPIO_SetBits(I2C_GPIO, I2C_SCL_PIN)
#define SCL_L()         GPIO_ResetBits(I2C_GPIO, I2C_SCL_PIN)
#define SDA_H()         GPIO_SetBits(I2C_GPIO, I2C_SDA_PIN)
#define SDA_L()         GPIO_ResetBits(I2C_GPIO, I2C_SDA_PIN)
#define SDA_READ()      GPIO_ReadInputDataBit(I2C_GPIO, I2C_SDA_PIN)

static volatile uint32_t g_ms_ticks = 0;

void SysTick_Handler(void)
{
    if (g_ms_ticks > 0) g_ms_ticks--;
}

static void Delay_ms(uint32_t ms)
{
    g_ms_ticks = ms;
    while (g_ms_ticks > 0) {}
}

static void I2C_Delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 24; i++) __NOP();
}

static void I2C_InitPins(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
    SCL_H();
    SDA_H();
    Delay_ms(2);
    SCL_L();
    Delay_ms(2);
    SCL_H();
    Delay_ms(1);
}

static void I2C_Start(void)
{
    SDA_H(); SCL_H(); I2C_Delay();
    SDA_L(); I2C_Delay();
    SCL_L(); I2C_Delay();
}

static void I2C_Stop(void)
{
    SDA_L(); I2C_Delay();
    SCL_H(); I2C_Delay();
    SDA_H(); I2C_Delay();
}

static uint8_t I2C_WriteByte(uint8_t data)
{
    uint8_t i;
    uint8_t ack;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) SDA_H(); else SDA_L();
        data <<= 1;
        I2C_Delay(); SCL_H(); I2C_Delay(); SCL_L(); I2C_Delay();
    }
    SDA_H(); I2C_Delay(); SCL_H(); I2C_Delay();
    ack = (SDA_READ() == 0) ? 1 : 0;
    SCL_L(); I2C_Delay();
    return ack;
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;
    SDA_H();
    for (i = 0; i < 8; i++) {
        data <<= 1;
        SCL_H(); I2C_Delay();
        if (SDA_READ()) data |= 0x01;
        SCL_L(); I2C_Delay();
    }
    if (ack) SDA_L(); else SDA_H();
    I2C_Delay(); SCL_H(); I2C_Delay(); SCL_L(); I2C_Delay(); SDA_H();
    return data;
}

static uint8_t I2C_Probe(uint8_t address)
{
    uint8_t ack;
    I2C_Start();
    ack = I2C_WriteByte(address);
    I2C_Stop();
    return ack;
}

static uint8_t MLX90614_ReadWord(uint8_t command, uint16_t *value)
{
    uint8_t low;
    uint8_t high;
    uint8_t pec;
    I2C_Start();
    if (!I2C_WriteByte(g_mlx_addr)) { I2C_Stop(); return 0; }
    if (!I2C_WriteByte(command)) { I2C_Stop(); return 0; }
    I2C_Start();
    if (!I2C_WriteByte(g_mlx_addr | 0x01)) { I2C_Stop(); return 0; }
    low = I2C_ReadByte(1);
    high = I2C_ReadByte(1);
    pec = I2C_ReadByte(0);
    (void)pec;
    I2C_Stop();
    *value = ((uint16_t)high << 8) | low;
    return 1;
}

static float MLX90614_RawToCelsius(uint16_t raw)
{
    return ((float)raw * 0.02f) - 273.15f;
}

static void OLED_WriteCommand(uint8_t command)
{
    I2C_Start();
    I2C_WriteByte(OLED_ADDR_3C);
    I2C_WriteByte(0x00);
    I2C_WriteByte(command);
    I2C_Stop();

    I2C_Start();
    I2C_WriteByte(OLED_ADDR_3D);
    I2C_WriteByte(0x00);
    I2C_WriteByte(command);
    I2C_Stop();
}

static void OLED_WriteData(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    I2C_Start();
    I2C_WriteByte(OLED_ADDR_3C);
    I2C_WriteByte(0x40);
    for (i = 0; i < length; i++) I2C_WriteByte(data[i]);
    I2C_Stop();

    I2C_Start();
    I2C_WriteByte(OLED_ADDR_3D);
    I2C_WriteByte(0x40);
    for (i = 0; i < length; i++) I2C_WriteByte(data[i]);
    I2C_Stop();
}

static void OLED_SetCursor(uint8_t x, uint8_t page)
{
    OLED_WriteCommand((uint8_t)(0xB0 + page));
    OLED_WriteCommand((uint8_t)(0x00 + (x & 0x0F)));
    OLED_WriteCommand((uint8_t)(0x10 + ((x >> 4) & 0x0F)));
}

static void OLED_Clear(void)
{
    uint8_t page;
    uint8_t zeros[128];
    uint16_t i;
    for (i = 0; i < sizeof(zeros); i++) zeros[i] = 0x00;
    for (page = 0; page < 8; page++) {
        OLED_SetCursor(0, page);
        OLED_WriteData(zeros, sizeof(zeros));
    }
}

static uint8_t OLED_Init(void)
{
    g_oled_addr = OLED_ADDR_3C;
    Delay_ms(100);
    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0x20);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x2E);
    OLED_WriteCommand(0xAF);
    OLED_Clear();
    return 1;
}

static const uint8_t *OLED_GetGlyph(char c)
{
    static const uint8_t digit0[5] = {0x3E,0x51,0x49,0x45,0x3E};
    static const uint8_t digit1[5] = {0x00,0x42,0x7F,0x40,0x00};
    static const uint8_t digit2[5] = {0x42,0x61,0x51,0x49,0x46};
    static const uint8_t digit3[5] = {0x21,0x41,0x45,0x4B,0x31};
    static const uint8_t digit4[5] = {0x18,0x14,0x12,0x7F,0x10};
    static const uint8_t digit5[5] = {0x27,0x45,0x45,0x45,0x39};
    static const uint8_t digit6[5] = {0x3C,0x4A,0x49,0x49,0x30};
    static const uint8_t digit7[5] = {0x01,0x71,0x09,0x05,0x03};
    static const uint8_t digit8[5] = {0x36,0x49,0x49,0x49,0x36};
    static const uint8_t digit9[5] = {0x06,0x49,0x49,0x29,0x1E};
    static const uint8_t dot[5]    = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t minus[5]  = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t equal[5]  = {0x14,0x14,0x14,0x14,0x14};
    static const uint8_t colon[5]  = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t letterA[5]= {0x7E,0x11,0x11,0x11,0x7E};
    static const uint8_t letterC[5]= {0x3E,0x41,0x41,0x41,0x22};
    static const uint8_t letterE[5]= {0x7F,0x49,0x49,0x49,0x41};
    static const uint8_t letterL[5]= {0x7F,0x40,0x40,0x40,0x40};
    static const uint8_t letterM[5]= {0x7F,0x02,0x0C,0x02,0x7F};
    static const uint8_t letterO[5]= {0x3E,0x41,0x41,0x41,0x3E};
    static const uint8_t letterR[5]= {0x7F,0x09,0x19,0x29,0x46};
    static const uint8_t letterT[5]= {0x01,0x01,0x7F,0x01,0x01};
    static const uint8_t letterX[5]= {0x63,0x14,0x08,0x14,0x63};
    static const uint8_t space[5]  = {0x00,0x00,0x00,0x00,0x00};
    switch (c) {
        case '0': return digit0;
        case '1': return digit1;
        case '2': return digit2;
        case '3': return digit3;
        case '4': return digit4;
        case '5': return digit5;
        case '6': return digit6;
        case '7': return digit7;
        case '8': return digit8;
        case '9': return digit9;
        case '.': return dot;
        case '-': return minus;
        case '=': return equal;
        case ':': return colon;
        case 'A': return letterA;
        case 'C': return letterC;
        case 'E': return letterE;
        case 'L': return letterL;
        case 'M': return letterM;
        case 'O': return letterO;
        case 'R': return letterR;
        case 'T': return letterT;
        case 'X': return letterX;
        default:  return space;
    }
}

static void OLED_ShowString(uint8_t x, uint8_t page, const char *text)
{
    uint8_t data[6];
    uint8_t i;
    OLED_SetCursor(x, page);
    while (*text) {
        const uint8_t *glyph = OLED_GetGlyph(*text++);
        for (i = 0; i < 5; i++) data[i] = glyph[i];
        data[5] = 0x00;
        OLED_WriteData(data, 6);
    }
}

static void OLED_ClearPage(uint8_t page)
{
    uint8_t zeros[128];
    uint16_t i;
    for (i = 0; i < sizeof(zeros); i++) zeros[i] = 0x00;
    OLED_SetCursor(0, page);
    OLED_WriteData(zeros, sizeof(zeros));
}


static const uint8_t cjk_76EE[32] = {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x3F,0x32,0x11,0x32,0x11,0x32,0x11,0x32,0x11,0x32,0x11,0x32,0x11,0x32,0x11,0x32,0x11,0xFE,0x3F,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t cjk_6807[32] = {0x00,0x00,0x30,0x0C,0xB0,0x03,0xFE,0x7F,0xB0,0x00,0x30,0x03,0x40,0x32,0x44,0x1E,0x44,0x46,0x44,0x40,0xC4,0x7F,0x44,0x00,0x44,0x02,0x44,0x0E,0x40,0x18,0x00,0x00};
static const uint8_t cjk_6E29[32] = {0x00,0x00,0x20,0x60,0x44,0x3C,0x4C,0x46,0x00,0x40,0x00,0x7E,0xFC,0x42,0x94,0x42,0x94,0x7E,0x94,0x42,0x94,0x7E,0x94,0x42,0xFC,0x42,0x00,0x7E,0x00,0x40,0x00,0x00};
static const uint8_t cjk_5EA6[32] = {0x00,0x00,0x00,0x30,0xFC,0x1F,0x04,0x00,0x24,0x42,0x24,0x22,0xFC,0x26,0xA4,0x2E,0xA7,0x1A,0xA4,0x12,0xA4,0x2A,0xFC,0x26,0x24,0x22,0x24,0x20,0x00,0x60,0x00,0x00};
static const uint8_t cjk_73AF[32] = {0x00,0x00,0x22,0x0C,0x22,0x04,0xFE,0x07,0x22,0x02,0x22,0x02,0x00,0x0C,0x02,0x06,0x82,0x01,0xE2,0x00,0xFE,0x7F,0x06,0x00,0x62,0x00,0x82,0x01,0x02,0x03,0x00,0x00};
static const uint8_t cjk_5883[32] = {0x00,0x00,0x10,0x08,0xFF,0x07,0x10,0x04,0x10,0x64,0x10,0x20,0xD2,0x27,0x52,0x15,0x5A,0x0D,0x53,0x05,0x53,0x3D,0x5E,0x25,0x52,0x25,0xD2,0x27,0x10,0x30,0x00,0x00};

static const uint8_t cjk_62A5[32] = {0x00,0x00,0x18,0x01,0x18,0x21,0x98,0x21,0xFF,0x3F,0x98,0x00,0x98,0x00,0xFE,0x3F,0x42,0x20,0xC2,0x21,0x52,0x17,0x52,0x0C,0x5A,0x1F,0xDE,0x31,0x00,0x60,0x00,0x00};
static const uint8_t cjk_8B66[32] = {0x00,0x00,0x92,0x00,0xFA,0x00,0xDF,0x76,0xDA,0x36,0xFA,0x36,0xAF,0x36,0xAA,0x36,0x9A,0x36,0xC4,0x36,0xDB,0x36,0xB3,0x36,0xBA,0x76,0xE6,0x00,0xC2,0x00,0x00,0x00};
static const uint8_t cjk_9608[32] = {0x00,0x00,0xF8,0x3F,0xF9,0x3F,0x13,0x08,0xD2,0x0B,0x50,0x0A,0xD2,0x0B,0x52,0x14,0x7E,0x18,0x92,0x0F,0x96,0x0F,0x5E,0x18,0x12,0x28,0x02,0x20,0xFE,0x3F,0x00,0x00};
static const uint8_t cjk_503C[32] = {0x00,0x00,0xC0,0x01,0x60,0x00,0xF8,0x7F,0x0E,0x00,0x00,0x40,0xE8,0x7F,0xE8,0x7F,0xA8,0x4A,0xBE,0x4A,0xAE,0x4A,0xA8,0x4A,0xE8,0x7F,0x08,0x40,0x08,0x40,0x00,0x00};
static const uint8_t *OLED_GetCJK(uint16_t code)
{
    switch (code) {
        case 0x76EE: return cjk_76EE;
        case 0x6807: return cjk_6807;
        case 0x6E29: return cjk_6E29;
        case 0x5EA6: return cjk_5EA6;
        case 0x73AF: return cjk_73AF;
        case 0x5883: return cjk_5883;
        case 0x62A5: return cjk_62A5;
        case 0x8B66: return cjk_8B66;
        case 0x9608: return cjk_9608;
        case 0x503C: return cjk_503C;
        default: return cjk_76EE;
    }
}

static void OLED_ShowCJK16(uint8_t x, uint8_t page, uint16_t code)
{
    const uint8_t *g = OLED_GetCJK(code);
    uint8_t i;
    uint8_t data[16];
    for (i = 0; i < 16; i++) data[i] = g[i * 2];
    OLED_SetCursor(x, page);
    OLED_WriteData(data, 16);
    for (i = 0; i < 16; i++) data[i] = g[i * 2 + 1];
    OLED_SetCursor(x, page + 1);
    OLED_WriteData(data, 16);
}

static void OLED_ShowChineseTemp(uint8_t page, uint16_t c1, uint16_t c2, uint16_t c3, uint16_t c4, float value)
{
    char buffer[16];
    uint8_t index = 0;
    int16_t value10;
    int16_t whole;
    uint8_t frac;
    static uint8_t last_valid[8] = {0};
    static int16_t last_value10[8] = {0};
    if (value >= 0.0f) value10 = (int16_t)(value * 10.0f + 0.5f);
    else value10 = (int16_t)(value * 10.0f - 0.5f);
    /* 只有显示值变化时才重画，避免OLED不断清屏闪烁 */
    if (page < 8 && last_valid[page] && last_value10[page] == value10) return;
    if (page < 8) { last_valid[page] = 1; last_value10[page] = value10; }
    if (value10 < 0) { buffer[index++] = '-'; value10 = -value10; }
    whole = value10 / 10;
    frac = (uint8_t)(value10 % 10);
    if (whole >= 100) buffer[index++] = (char)('0' + (whole / 100) % 10);
    if (whole >= 10) buffer[index++] = (char)('0' + (whole / 10) % 10);
    buffer[index++] = (char)('0' + whole % 10);
    buffer[index++] = '.';
    buffer[index++] = (char)('0' + frac);
    buffer[index++] = 'C';
    buffer[index] = '\0';
    OLED_ClearPage(page); OLED_ClearPage(page + 1);
    OLED_ShowCJK16(0, page, c1);
    OLED_ShowCJK16(18, page, c2);
    OLED_ShowCJK16(36, page, c3);
    OLED_ShowCJK16(54, page, c4);
    OLED_ShowString(76, page + 1, buffer);
}

static void OLED_ShowTemperature(uint8_t page, const char *name, float value)
{
    char buffer[16];
    uint8_t index = 0;
    int16_t value10;
    int16_t whole;
    uint8_t frac;
    if (value >= 0.0f) value10 = (int16_t)(value * 10.0f + 0.5f);
    else value10 = (int16_t)(value * 10.0f - 0.5f);
    buffer[index++] = name[0];
    buffer[index++] = name[1];
    buffer[index++] = '=';
    if (value10 < 0) { buffer[index++] = '-'; value10 = -value10; }
    whole = value10 / 10;
    frac = (uint8_t)(value10 % 10);
    if (whole >= 100) buffer[index++] = (char)('0' + (whole / 100) % 10);
    if (whole >= 10) buffer[index++] = (char)('0' + (whole / 10) % 10);
    buffer[index++] = (char)('0' + whole % 10);
    buffer[index++] = '.';
    buffer[index++] = (char)('0' + frac);
    buffer[index++] = 'C';
    buffer[index] = '\0';
    OLED_ClearPage(page);
    OLED_ShowString(0, page, buffer);
}

static void Buzzer_Init(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = BUZZER_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUZZER_GPIO, &gpio);
    GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN);
}

#define KEY_ID_PIN      GPIO_Pin_0
#define KEY_VERIFY_PIN  GPIO_Pin_1
#define KEY_ENROLL_PIN  GPIO_Pin_2
#define KEY_TEMP_UP_PIN   GPIO_Pin_0  /* PB0：报警阈值 +0.5℃ */
#define KEY_TEMP_DOWN_PIN GPIO_Pin_1  /* PB1：报警阈值 -0.5℃ */

#define UART_RX_BUF_SIZE 128
#define UART_LINE_SIZE   64

typedef enum {
    ID_STATUS_READY = 0,
    ID_STATUS_WAIT,
    ID_STATUS_OK,
    ID_STATUS_MISMATCH,
    ID_STATUS_UNKNOWN,
    ID_STATUS_ENROLL_OK,
    ID_STATUS_ENROLL_FAIL
} IdStatus_t;

static volatile uint8_t g_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;

static IdStatus_t g_id_status = ID_STATUS_READY;
static int g_selected_id = 1;
static int g_actual_id = -1;
static int g_match_score10 = 0;
static int g_mask_status = -1;
static int g_mask_conf10 = 0;

static uint8_t g_key_id_last = 0;
static uint8_t g_key_verify_last = 0;
static uint8_t g_key_enroll_last = 0;
static uint8_t g_key_temp_up_last = 0;    /* 上一次PB0状态，用于判断按下边沿 */
static uint8_t g_key_temp_down_last = 0;  /* 上一次PB1状态，用于判断按下边沿 */

static int ParseScore10(const char *s)
{
    int whole = 0;
    int frac = 0;
    while (*s >= '0' && *s <= '9') {
        whole = whole * 10 + (*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        if (*s >= '0' && *s <= '9') frac = *s - '0';
    }
    return whole * 10 + frac;
}

static void USART1_SendString(const char *text)
{
    while (*text) {
        while ((USART1->SR & USART_SR_TXE) == 0) { }
        USART1->DR = (uint16_t)(*text++);
    }
}

static void USART1_SendIdCommand(const char *cmd, int id)
{
    char buffer[24];
    sprintf(buffer, "%s,%d\n", cmd, id);
    USART1_SendString(buffer);
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t data = (uint8_t)(USART1->DR & 0xFF);
        uint16_t next = (uint16_t)((g_rx_head + 1) % UART_RX_BUF_SIZE);
        if (next != g_rx_tail) {
            g_rx_buf[g_rx_head] = data;
            g_rx_head = next;
        }
    }
}

static void USART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    GPIOA->CRH &= ~(0xF << 4);
    GPIOA->CRH |=  (0xB << 4);      /* PA9 TX, AF push-pull, 50MHz */
    GPIOA->CRH &= ~(0xF << 8);
    GPIOA->CRH |=  (0x4 << 8);      /* PA10 RX, floating input */

    USART1->BRR = (SystemCoreClock + (115200 / 2)) / 115200;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

static void Keys_Init(void)
{
    GPIO_InitTypeDef gpio;
    /* PA0/PA1/PA2：身份按键；PB0/PB1：报警阈值加减键 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = KEY_ID_PIN | KEY_VERIFY_PIN | KEY_ENROLL_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;   /* 内部上拉，按键另一端接GND */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = KEY_TEMP_UP_PIN | KEY_TEMP_DOWN_PIN;
    GPIO_Init(GPIOB, &gpio);         /* PB0/PB1同样使用内部上拉输入 */
}

static uint8_t Key_ReadPort(GPIO_TypeDef *port, uint16_t pin)
{
    /* 按键接GND并启用内部上拉：松开=0，按下=1 */
    return (GPIO_ReadInputDataBit(port, pin) == Bit_RESET) ? 1 : 0;
}
static uint8_t Key_Read(uint16_t pin)
{
    return Key_ReadPort(GPIOA, pin);
}

static void K210_ParseLine(char *line)
{
    char *p;

    if (strncmp(line, "PONG", 4) == 0) {
        return;
    } else if (strncmp(line, "ID_OK,", 6) == 0) {
        p = line + 6;
        g_actual_id = atoi(p);
        p = strchr(p, ',');
        if (p) g_match_score10 = ParseScore10(p + 1);
        g_id_status = ID_STATUS_OK;
    } else if (strncmp(line, "ID_MISMATCH,", 12) == 0) {
        p = line + 12;
        g_selected_id = atoi(p);
        p = strchr(p, ',');
        if (p) {
            g_actual_id = atoi(p + 1);
            p = strchr(p + 1, ',');
        }
        if (p) g_match_score10 = ParseScore10(p + 1);
        g_id_status = ID_STATUS_MISMATCH;
    } else if (strncmp(line, "UNKNOWN,", 8) == 0) {
        p = line + 8;
        g_selected_id = atoi(p);
        p = strchr(p, ',');
        if (p) g_match_score10 = ParseScore10(p + 1);
        g_actual_id = -1;
        g_id_status = ID_STATUS_UNKNOWN;
    } else if (strncmp(line, "ENROLL_OK,", 10) == 0) {
        g_selected_id = atoi(line + 10);
        g_id_status = ID_STATUS_ENROLL_OK;
    } else if (strncmp(line, "ENROLL_FAIL,", 12) == 0) {
        g_id_status = ID_STATUS_ENROLL_FAIL;
    } else if (strncmp(line, "MASK_OK,", 8) == 0) {
        g_mask_conf10 = ParseScore10(line + 8);
        g_mask_status = 1;
    } else if (strncmp(line, "NO_MASK,", 8) == 0) {
        g_mask_conf10 = ParseScore10(line + 8);
        g_mask_status = 0;
    } else if (strncmp(line, "MASK_UNKNOWN", 12) == 0) {
        g_mask_status = -1;
    }
}

static void USART1_Process(void)
{
    static char line[UART_LINE_SIZE];
    static uint8_t len = 0;

    while (g_rx_tail != g_rx_head) {
        char c = (char)g_rx_buf[g_rx_tail];
        g_rx_tail = (uint16_t)((g_rx_tail + 1) % UART_RX_BUF_SIZE);

        if (c == '\r') {
            continue;
        } else if (c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                K210_ParseLine(line);
                len = 0;
            }
        } else if (len < UART_LINE_SIZE - 1) {
            line[len++] = c;
        } else {
            len = 0;
        }
    }
}

static void Keys_Process(void)
{
    uint8_t k_id = Key_Read(KEY_ID_PIN);
    uint8_t k_verify = Key_Read(KEY_VERIFY_PIN);
    uint8_t k_enroll = Key_Read(KEY_ENROLL_PIN);
    /* 读取PB0/PB1当前逻辑状态：0=松开，1=按下 */
    uint8_t k_temp_up = Key_ReadPort(GPIOB, KEY_TEMP_UP_PIN);
    uint8_t k_temp_down = Key_ReadPort(GPIOB, KEY_TEMP_DOWN_PIN);

    if (k_id && !g_key_id_last) {
        g_selected_id++;
        if (g_selected_id > 10) g_selected_id = 1;
        g_id_status = ID_STATUS_READY;
        g_mask_status = -1;
        g_actual_id = -1;
        g_match_score10 = 0;
    }

    if (k_verify && !g_key_verify_last) {
        g_id_status = ID_STATUS_WAIT;
        g_mask_status = -1;
        g_actual_id = -1;
        g_match_score10 = 0;
        USART1_SendIdCommand("VERIFY", g_selected_id);
    }

    if (k_enroll && !g_key_enroll_last) {
        g_id_status = ID_STATUS_WAIT;
        g_mask_status = -1;
        g_actual_id = -1;
        g_match_score10 = 0;
        USART1_SendIdCommand("ENROLL", g_selected_id);
    }

    /* PB0新按下一次：报警阈值 +0.5℃；ALARM_TEMP_MAX_10限制最高45.0℃ */
    if (k_temp_up && !g_key_temp_up_last && g_alarm_temp_10 < ALARM_TEMP_MAX_10) {
        g_alarm_temp_10 += 5;
    }
    /* PB1新按下一次：报警阈值 -0.5℃；ALARM_TEMP_MIN_10限制最低30.0℃ */
    if (k_temp_down && !g_key_temp_down_last && g_alarm_temp_10 > ALARM_TEMP_MIN_10) {
        g_alarm_temp_10 -= 5;
    }
    /* 保存本次状态，下一次循环用于判断是否出现新的按下边沿 */
    g_key_id_last = k_id;
    g_key_verify_last = k_verify;
    g_key_enroll_last = k_enroll;
    g_key_temp_up_last = k_temp_up;
    g_key_temp_down_last = k_temp_down;
}

static void OLED_ShowK210Status(void)
{
    if (!g_oled_ok) return;
    char line1[24];
    char line2[24];
    static char last_line1[24] = {0};
    static char last_line2[24] = {0};
    const char *status;
    const char *mask;

    switch (g_id_status) {
        case ID_STATUS_WAIT:        status = "WAIT";  break;
        case ID_STATUS_OK:          status = "OK";    break;
        case ID_STATUS_MISMATCH:    status = "BAD";   break;
        case ID_STATUS_UNKNOWN:     status = "UNK";   break;
        case ID_STATUS_ENROLL_OK:   status = "SAVE";  break;
        case ID_STATUS_ENROLL_FAIL: status = "FAIL";  break;
        default:                    status = "RDY";   break;
    }

    if (g_mask_status == 1) mask = "M:Y";
    else if (g_mask_status == 0) mask = "M:N";
    else mask = "M:?";

    sprintf(line1, "ID%d %s %s", g_selected_id, status, mask);
    sprintf(line2, "S:%d.%d A:%d", g_match_score10 / 10, g_match_score10 % 10, g_actual_id);
    if (strcmp(line1, last_line1) == 0 && strcmp(line2, last_line2) == 0) return;
    strcpy(last_line1, line1);
    strcpy(last_line2, line2);

    OLED_ClearPage(6);
    OLED_ShowString(0, 6, line1);
    OLED_ClearPage(7);
    OLED_ShowString(0, 7, line2);
}

static uint8_t Alarm_Check(float target_temp, uint8_t temp_valid)
{
    /* 目标温度超过当前报警阈值时报警 */
    if (temp_valid && target_temp > (g_alarm_temp_10 / 10.0f)) return 1;
    if (g_id_status == ID_STATUS_MISMATCH) return 1;
    if (g_id_status == ID_STATUS_UNKNOWN) return 1;
    if (g_mask_status == 0) return 1;
    return 0;
}


int main(void)
{
    uint16_t raw_to;
    uint16_t raw_ta;
    float to = 0.0f;
    float ta = 0.0f;

    SysTick_Config(SystemCoreClock / 1000);

    I2C_InitPins();
    Buzzer_Init();
    Keys_Init();
    USART1_Init();

    Delay_ms(300);
    g_oled_ok = OLED_Init();

    if (I2C_Probe(MLX_ADDR_00)) {
        g_mlx_addr = MLX_ADDR_00;
        g_mlx_ok = 1;
    } else if (I2C_Probe(MLX_ADDR_5A)) {
        g_mlx_addr = MLX_ADDR_5A;
        g_mlx_ok = 1;
    } else if (I2C_Probe(MLX_ADDR_5B)) {
        g_mlx_addr = MLX_ADDR_5B;
        g_mlx_ok = 1;
    } else {
        g_mlx_ok = 0;
    }

    USART1_SendString("PING\n");

    while (1) {
        USART1_Process();
        Keys_Process();  /* 每100ms处理一次按键，按下一次只改变一次阈值 */

        if (g_mlx_ok) {
            if (MLX90614_ReadWord(0x07, &raw_to) && MLX90614_ReadWord(0x06, &raw_ta)) {
                to = MLX90614_RawToCelsius(raw_to);
                ta = MLX90614_RawToCelsius(raw_ta);
                if (g_oled_ok) {
                    OLED_ShowChineseTemp(0, 0x76EE, 0x6807, 0x6E29, 0x5EA6, to);
                    OLED_ShowChineseTemp(2, 0x73AF, 0x5883, 0x6E29, 0x5EA6, ta);
                }
            } else {
                g_mlx_ok = 0;
            }
        }

        if (!g_mlx_ok && g_oled_ok) {
            static uint8_t mlx_err_shown = 0;
            if (!mlx_err_shown) {
                OLED_ClearPage(0);
                OLED_ShowString(0, 0, "MLX ERR");
                mlx_err_shown = 1;
            }
        }

        if (g_oled_ok) {
            /* 显示当前报警阈值，g_alarm_temp_10除以10转换为℃ */
            OLED_ShowChineseTemp(4, 0x62A5, 0x8B66, 0x9608, 0x503C, g_alarm_temp_10 / 10.0f);
        }

        if (Alarm_Check(to, g_mlx_ok)) GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
        else GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN);

        Delay_ms(100);
    }
}
