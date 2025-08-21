#include <math.h>
#include <oled.h>
#include <stdlib.h>
#include <string.h>

#if defined(OLED_USE_I2C)

void oled_Reset(void) {
    /* za I2C - nema reset pina, ništa se ne radi */
}

// Pošalji jedan bajt u registar za naredbe
void oled_WriteCommand(uint8_t byte) {
    HAL_I2C_Mem_Write(&OLED_I2C_PORT, OLED_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

// Pošalji podatke
void oled_WriteData(uint8_t* buffer, size_t buff_size) {
    HAL_I2C_Mem_Write(&OLED_I2C_PORT, OLED_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}

#endif


// Framebuffer ekrana
static uint8_t OLED_Buffer[OLED_BUFFER_SIZE];

// Struktura stanja ekrana
static OLED_t OLED;

/* Inicijalizacija OLED ekrana */
void oled_Init(void) {
    // Resetiraj OLED (ako je potrebno)
    oled_Reset();

    // Pričekaj da se ekran pokrene
    HAL_Delay(100);

    // Inicijalizacija OLED-a
    oled_SetDisplayOn(0); // Isključi prikaz

    oled_WriteCommand(0x20); // Odabir načina adresiranja memorije
    oled_WriteCommand(0x00); // 00b = horizontalni, 01b = vertikalni,
                             // 10b = page addressing (default), 11b = neispravno

    oled_WriteCommand(0xB0); // Početna stranica (za page addressing način)

#ifdef OLED_MIRROR_VERT
    oled_WriteCommand(0xC0); // Zrcali okomito
#else
    oled_WriteCommand(0xC8); // Smjer skeniranja COM linija
#endif

    oled_WriteCommand(0x00); // Donja kolona = 0
    oled_WriteCommand(0x10); // Gornja kolona = 0

    oled_WriteCommand(0x40); // Početna linija = 0

    oled_SetContrast(0xFF); // Maksimalni kontrast

#ifdef OLED_MIRROR_HORIZ
    oled_WriteCommand(0xA0); // Zrcali vodoravno
#else
    oled_WriteCommand(0xA1); // Normalni remap segmenata
#endif

#ifdef OLED_INVERSE_COLOR
    oled_WriteCommand(0xA7); // Inverzne boje
#else
    oled_WriteCommand(0xA6); // Normalne boje
#endif

// Postavi multiplex omjer
#if (OLED_HEIGHT == 128)
    oled_WriteCommand(0xFF);
#else
    oled_WriteCommand(0xA8); // Postavi multiplex omjer (1–64)
#endif

#if (OLED_HEIGHT == 32)
    oled_WriteCommand(0x1F);
#elif (OLED_HEIGHT == 64)
    oled_WriteCommand(0x3F);
#elif (OLED_HEIGHT == 128)
    oled_WriteCommand(0x3F); // Radi i za 128px visoke ekrane
#else
#error "Podržano je samo 32, 64 ili 128 piksela u visini!"
#endif

    oled_WriteCommand(0xA4); // koristi RAM sadržaj

    oled_WriteCommand(0xD3); // Offset prikaza
    oled_WriteCommand(0x00); // Bez offseta

    oled_WriteCommand(0xD5); // Postavi brzinu takta / frekvenciju oscilatora
    oled_WriteCommand(0xF0); // Vrijednost djelitelja

    oled_WriteCommand(0xD9); // Pre-charge period
    oled_WriteCommand(0x22);

    oled_WriteCommand(0xDA); // Konfiguracija COM pinova
#if (OLED_HEIGHT == 32)
    oled_WriteCommand(0x02);
#elif (OLED_HEIGHT == 64)
    oled_WriteCommand(0x12);
#elif (OLED_HEIGHT == 128)
    oled_WriteCommand(0x12);
#else
#error "Podržano je samo 32, 64 ili 128 piksela u visini!"
#endif

    oled_WriteCommand(0xDB); // VCOMH
    oled_WriteCommand(0x20); // 0.77 × Vcc

    oled_WriteCommand(0x8D); // Uključi DC-DC
    oled_WriteCommand(0x14);

    oled_SetDisplayOn(1); // Uključi prikaz

    // Očisti ekran
    oled_Fill(Black);
    
    // Pošalji prazan framebuffer na ekran
    oled_UpdateScreen();
    
    // Postavi početne vrijednosti u strukturi
    OLED.CurrentX = 0;
    OLED.CurrentY = 0;
    OLED.Initialized = 1;
}

void oled_Fill(OLED_COLOR color) {
    memset(OLED_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(OLED_Buffer));
}

void oled_UpdateScreen(void) {
    // Piši podatke u svaku stranicu RAM-a.
    // Broj stranica ovisi o visini ekrana:
    //
    //  * 32 px   = 4 stranice
    //  * 64 px   = 8 stranica
    //  * 128 px  = 16 stranica
    for(uint8_t i = 0; i < OLED_HEIGHT/8; i++) {
        oled_WriteCommand(0xB0 + i); // Trenutna stranica
        oled_WriteCommand(0x00 + OLED_X_OFFSET_LOWER);
        oled_WriteCommand(0x10 + OLED_X_OFFSET_UPPER);
        oled_WriteData(&OLED_Buffer[OLED_WIDTH*i], OLED_WIDTH);
    }
}

void oled_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color) {
    if(x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        // Nemoj crtati izvan buffera
        return;
    }
   
    // Postavi piksel u zadanoj boji
    if(color == White) {
        OLED_Buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
    } else { 
        OLED_Buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

char oled_WriteChar(char ch, OLED_Font_t Font, OLED_COLOR color) {
    uint32_t i, b, j;
    
    // Provjeri je li znak u ASCII rasponu
    if (ch < 32 || ch > 126)
        return 0;
    
    // Širina znaka (razlikuje se za proporcionalne fontove)
    const uint8_t char_width = Font.char_width ? Font.char_width[ch-32] : Font.width;

    // Provjera ima li mjesta u retku
    if (OLED_WIDTH < (OLED.CurrentX + char_width) ||
        OLED_HEIGHT < (OLED.CurrentY + Font.height)) {
        return 0; // Nema mjesta
    }
    
    // Iscrtavanje znaka iz font tablice
    for(i = 0; i < Font.height; i++) {
        b = Font.data[(ch - 32) * Font.height + i];
        for(j = 0; j < char_width; j++) {
            if((b << j) & 0x8000)  {
                oled_DrawPixel(OLED.CurrentX + j, (OLED.CurrentY + i), (OLED_COLOR) color);
            } else {
                oled_DrawPixel(OLED.CurrentX + j, (OLED.CurrentY + i), (OLED_COLOR)!color);
            }
        }
    }
    
    // Pomičemo kursor na kraj znaka
    OLED.CurrentX += char_width;
    
    return ch; // vraća ispisani znak
}

/* Ispis niza znakova na ekran */
char oled_WriteString(char* str, OLED_Font_t Font, OLED_COLOR color) {
    while (*str) {
        if (oled_WriteChar(*str, Font, color) != *str) {
            return *str; // neki znak nije upisan
        }
        str++;
    }
    return *str;
}

/* Postavi kursor */
void oled_SetCursor(uint8_t x, uint8_t y) {
    OLED.CurrentX = x;
    OLED.CurrentY = y;
}

/* Postavi kontrast */
void oled_SetContrast(const uint8_t value) {
    const uint8_t kSetContrastControlRegister = 0x81;
    oled_WriteCommand(kSetContrastControlRegister);
    oled_WriteCommand(value);
}

/* Uključi ili isključi prikaz */
void oled_SetDisplayOn(const uint8_t on) {
    uint8_t value;
    if (on) {
        value = 0xAF;   // Ekran uključen
        OLED.DisplayOn = 1;
    } else {
        value = 0xAE;   // Ekran isključen
        OLED.DisplayOn = 0;
    }
    oled_WriteCommand(value);
}

/* Vrati trenutno stanje prikaza (1 = uključen, 0 = isključen) */
uint8_t oled_GetDisplayOn() {
    return OLED.DisplayOn;
}
