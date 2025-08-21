#ifndef __OLED_CONF_H__
#define __OLED_CONF_H__

// Ime platforme
#define STM32F4

// Odabir sabirnice za komunikaciju
#define OLED_USE_I2C   // koristi I2C

// Konfiguracija I2C-a
#define OLED_I2C_PORT        hi2c1           // I2C port koji koristimo
#define OLED_I2C_ADDR        (0x3C << 1)     // I2C adresa OLED modula

// Font
#define OLED_INCLUDE_FONT_6x8   // uključi 6x8 font

// Dimenzije ekrana
#define OLED_WIDTH           128   // širina u pikselima
#define OLED_HEIGHT          64    // visina u pikselima

#endif /* __OLED_CONF_H__ */
