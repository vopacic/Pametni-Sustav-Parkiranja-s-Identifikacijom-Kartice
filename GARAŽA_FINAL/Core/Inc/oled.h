#ifndef __OLED_H__
#define __OLED_H__

#include <stddef.h>
#include <stdint.h>
#include <_ansi.h>

_BEGIN_STD_C

#include <oled_conf.h>

#if defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#endif

#ifdef OLED_X_OFFSET
#define OLED_X_OFFSET_LOWER (OLED_X_OFFSET & 0x0F)
#define OLED_X_OFFSET_UPPER ((OLED_X_OFFSET >> 4) & 0x07)
#else
#define OLED_X_OFFSET_LOWER 0
#define OLED_X_OFFSET_UPPER 0
#endif

// Konfiguracija I2C-a

#ifndef OLED_I2C_PORT
#define OLED_I2C_PORT        hi2c1
#endif

#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR        (0x3C << 1)
#endif

#if defined(OLED_USE_I2C)
extern I2C_HandleTypeDef OLED_I2C_PORT;
#endif

// Visina OLED ekrana u pikselima
#ifndef OLED_HEIGHT
#define OLED_HEIGHT          64
#endif

// Širina OLED ekrana u pikselima
#ifndef OLED_WIDTH
#define OLED_WIDTH           128
#endif

#ifndef OLED_BUFFER_SIZE
#define OLED_BUFFER_SIZE   OLED_WIDTH * OLED_HEIGHT / 8
#endif

// Enumeracija za boje ekrana
typedef enum {
    Black = 0x00, // Crna boja (ugašen pixel)
    White = 0x01  // Bijela boja (upaljen pixel)
} OLED_COLOR;

// Statusne vrijednosti funkcija
typedef enum {
    OLED_OK = 0x00,
    OLED_ERR = 0x01  // Greška
} OLED_Error_t;

// Struktura za spremanje trenutnih koordinata i stanja
typedef struct {
    uint16_t CurrentX;   // Trenutna X pozicija kursora
    uint16_t CurrentY;   // Trenutna Y pozicija kursora
    uint8_t Initialized; // Zastavica inicijalizacije
    uint8_t DisplayOn;   // Zastavica stanja ekrana (uključen/isključen)
} OLED_t;

// Struktura za pohranu koordinata točke
typedef struct {
    uint8_t x;
    uint8_t y;
} OLED_VERTEX;

/** Font */
typedef struct {
	const uint8_t width;                /**< Širina fonta u pikselima */
	const uint8_t height;               /**< Visina fonta u pikselima */
	const uint16_t *const data;         /**< Pokazivač na polje s podacima fonta */
    const uint8_t *const char_width;    /**< Širina svakog znaka (NULL za monospaced fontove) */
} OLED_Font_t;

// Deklaracije glavnih funkcija
void oled_Init(void);
void oled_Fill(OLED_COLOR color);
void oled_UpdateScreen(void);
void oled_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color);
char oled_WriteChar(char ch, OLED_Font_t Font, OLED_COLOR color);
char oled_WriteString(char* str, OLED_Font_t Font, OLED_COLOR color);
void oled_SetCursor(uint8_t x, uint8_t y);
void oled_SetContrast(const uint8_t value);
void oled_SetDisplayOn(const uint8_t on);
uint8_t oled_GetDisplayOn();

// Osnovne funkcije
void oled_Reset(void);
void oled_WriteCommand(uint8_t byte);
void oled_WriteData(uint8_t* buffer, size_t buff_size);

_END_STD_C

#endif // __OLED_H__
