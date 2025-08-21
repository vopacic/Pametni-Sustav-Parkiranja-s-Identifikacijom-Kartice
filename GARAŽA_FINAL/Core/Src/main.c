#include "main.h"             // Glavna zaglavlja generirana od STM32CubeMX
#include "i2c.h"              // Inicijalizacija I2C periferije
#include "spi.h"              // Inicijalizacija SPI periferije
#include "tim.h"              // Tajmeri
#include "usart.h"            // Serijska komunikacija
#include "gpio.h"             // Rad s pinovima (input/output)
#include "string.h"           // Standardna string biblioteka (npr. memcpy)
#include "oled.h"          // OLED ekran kontrola
#include "oled_font.h"    // Fontovi za OLED ekran

//RFID

#include "stm32f4xx_hal.h"

#define	uchar	unsigned char
#define	uint	unsigned int
extern SPI_HandleTypeDef hspi1;

// Maksimalna duljina polja
#define MAX_LEN 16

#define HSPI_INSTANCE				&hspi1
#define MFRC522_CS_PORT				GPIOB
#define MFRC522_CS_PIN				GPIO_PIN_9
#define MFRC522_RST_PORT			GPIOC
#define MFRC522_RST_PIN				GPIO_PIN_7

// MFRC522 naredbe. Opisane u poglavlju 10 tehničkog lista.
#define PCD_IDLE              0x00               // bez radnje, otkazuje trenutno izvođenje naredbe
#define PCD_AUTHENT           0x0E               // izvodi MIFARE standardnu autentifikaciju kao čitač
#define PCD_RECEIVE           0x08               // aktivira krugove prijemnika
#define PCD_TRANSMIT          0x04               // prenosi podatke iz FIFO spremnika
#define PCD_TRANSCEIVE        0x0C               // prenosi podatke iz FIFO spremnika na antenu i automatski aktivira prijemnik nakon prijenosa
#define PCD_RESETPHASE        0x0F               // resetira MFRC522
#define PCD_CALCCRC           0x03               // aktivira CRC koprocesor ili izvodi samoprovjeru

// Naredbe poslane PICC-u.
#define PICC_REQIDL           0x26               // REQuest naredba, Tip A. Poziva PICC-eve u stanju IDLE da pređu u READY i pripreme se za antikolidž ili odabir. 7-bitni okvir.
#define PICC_REQALL           0x52               // Wake-UP naredba, Tip A. Poziva PICC-eve u stanju IDLE i HALT da pređu u READY(*) i pripreme se za antikolidž ili odabir. 7-bitni okvir.
#define PICC_ANTICOLL         0x93               // Antikolidž/Odabir, Kaskadna razina 1
#define PICC_SElECTTAG        0x93               // Antikolidž/Odabir, Kaskadna razina 2
#define PICC_AUTHENT1A        0x60               // Izvodi autentifikaciju s ključem A
#define PICC_AUTHENT1B        0x61               // Izvodi autentifikaciju s ključem B
#define PICC_READ             0x30               // Čita jedan 16-bajtni blok iz autentificiranog sektora PICC-a. Također se koristi za MIFARE Ultralight.
#define PICC_WRITE            0xA0               // Piše jedan 16-bajtni blok u autentificirani sektor PICC-a. Naziva se "COMPATIBILITY WRITE" za MIFARE Ultralight.
#define PICC_DECREMENT        0xC0               // Smanjuje sadržaj bloka i sprema rezultat u interni registar podataka.
#define PICC_INCREMENT        0xC1               // Povećava sadržaj bloka i sprema rezultat u interni registar podataka.
#define PICC_RESTORE          0xC2               // Čita sadržaj bloka u interni registar podataka.
#define PICC_TRANSFER         0xB0               // Piše sadržaj internog registra podataka u blok.
#define PICC_HALT             0x50               // HaLT naredba, Tip A. Naređuje AKTIVNOM PICC-u da pređe u stanje HALT.

// Vraća se kod uspjeha ili greške pri komunikaciji
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

// MFRC522 registri. Opisani u poglavlju 9 tehničkog lista.
// Stranica 0: Naredbe i status
#define     Reserved00            0x00
#define     CommandReg            0x01
#define     CommIEnReg            0x02
#define     DivlEnReg             0x03
#define     CommIrqReg            0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
// Stranica 1: Naredbe
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
// Stranica 2: Konfiguracija
#define     Reserved20            0x20
#define     CRCResultRegH         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg	          0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// Stranica 3: Testni registri
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     Reserved31            0x3C
#define     Reserved32            0x3D
#define     Reserved33            0x3E
#define     Reserved34			  0x3F

// Funkcije za upravljanje MFRC522
void MFRC522_Init(void);
uchar MFRC522_Request(uchar reqMode, uchar *TagType);
uchar MFRC522_Anticoll(uchar *serNum);
uchar MFRC522_SelectTag(uchar *serNum);
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey,
		uchar *serNum);
uchar MFRC522_Write(uchar blockAddr, uchar *writeData);
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey,
		uchar *serNum);
uchar MFRC522_Read(uchar blockAddr, uchar *recvData);
void MFRC522_Halt(void);


/*
 * Naziv funkcije: RC522_SPI_Transfer
 * Opis: Slanje jednog bajta podataka preko SPI i istovremeno primanje odgovora.
 * Napomena: MFRC522 uvijek vraća neki podatak pri SPI prijenosu, zato koristimo TransmitReceive.
 */
uint8_t RC522_SPI_Transfer(uchar data) {
	uchar rx_data;
	HAL_SPI_TransmitReceive(HSPI_INSTANCE, &data, &rx_data, 1, 100); // Timeout 100ms - dovoljno za brze transakcije

	return rx_data;
}

/*
 * Naziv funkcije: Write_MFRC522
 * Opis: Upis vrijednosti u registar RC522.
 * Napomena: CS pin mora biti LOW prije slanja i HIGH nakon slanja da bi RC522 ispravno registrirao naredbu.
 */
void Write_MFRC522(uchar addr, uchar val) {
	/* Aktivacija SPI veze prema RC522 */
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET);

	/* Šaljemo adresu registra (shift lijevo, zadnji bit 0 = write) */
	RC522_SPI_Transfer((addr << 1) & 0x7E);
	/* Šaljemo vrijednost za upis */
	RC522_SPI_Transfer(val);

	/* Deaktivacija SPI veze */
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);
}

/*
 * Naziv funkcije: Read_MFRC522
 * Opis: Čitanje vrijednosti iz registra RC522.
 * Napomena: Zadnji bit adrese je 1 = read.
 */
uchar Read_MFRC522(uchar addr) {
	uchar val;

	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET);

	RC522_SPI_Transfer(((addr << 1) & 0x7E) | 0x80); // MSB=1 -> read mode
	val = RC522_SPI_Transfer(0x00); // Pošaljemo dummy bajt da dobijemo odgovor

	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);

	return val;
}

/*
 * Naziv funkcije: SetBitMask
 * Opis: Postavlja određene bitove u registru.
 * Koristi se za uključivanje pojedinih funkcija bez mijenjanja ostalih bitova.
 */
void SetBitMask(uchar reg, uchar mask) {
	uchar tmp = Read_MFRC522(reg);
	Write_MFRC522(reg, tmp | mask);
}

/*
 * Naziv funkcije: ClearBitMask
 * Opis: Briše određene bitove u registru.
 * Koristi se za gašenje pojedinih funkcija bez utjecaja na ostale bitove.
 */
void ClearBitMask(uchar reg, uchar mask) {
	uchar tmp = Read_MFRC522(reg);
	Write_MFRC522(reg, tmp & (~mask));
}

/*
 * Naziv funkcije: AntennaOn
 * Opis: Uključuje RF antenu.
 * Napomena: Potrebno je imati uključenu antenu prije komunikacije s karticom.
 */
void AntennaOn(void) {
	Read_MFRC522(TxControlReg); // Čitanje registra nije nužno, ali se često radi radi stabilnosti
	SetBitMask(TxControlReg, 0x03);
}

/*
 * Naziv funkcije: AntennaOff
 * Opis: Isključuje RF antenu.
 */
void AntennaOff(void) {
	ClearBitMask(TxControlReg, 0x03);
}

/*
 * Naziv funkcije: MFRC522_Reset
 * Opis: Resetira MFRC522 čip.
 * Napomena: Nakon reseta potrebno je ponovno inicijalizirati sve registre.
 */
void MFRC522_Reset(void) {
	Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

/*
 * Naziv funkcije: MFRC522_Init
 * Opis: Inicijalizira MFRC522 u radno stanje.
 * Napomena: Postavlja tajmere, modulaciju, CRC i uključuje antenu.
 */
void MFRC522_Init(void) {
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MFRC522_RST_PORT, MFRC522_RST_PIN, GPIO_PIN_SET);
	MFRC522_Reset();

	Write_MFRC522(TModeReg, 0x8D);
	Write_MFRC522(TPrescalerReg, 0x3E);
	Write_MFRC522(TReloadRegL, 30);
	Write_MFRC522(TReloadRegH, 0);

	Write_MFRC522(TxAutoReg, 0x40); // 100% ASK modulacija
	Write_MFRC522(ModeReg, 0x3D);   // CRC početna vrijednost 0x6363

	AntennaOn();
}

/*
 * Naziv funkcije: MFRC522_ToCard
 * Opis: Glavna funkcija za slanje i primanje podataka između RC522 i kartice.
 * Napomena: Koristi FIFO buffer čipa, upravlja prekidima i provjerava greške.
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen,
		uchar *backData, uint *backLen) {
	uchar status = MI_ERR;
	uchar irqEn = 0x00;
	uchar waitIRq = 0x00;
	uchar lastBits;
	uchar n;
	uint i;

	/* Određivanje prekida i čekanja ovisno o naredbi */
	switch (command) {
	case PCD_AUTHENT:
		irqEn = 0x12;
		waitIRq = 0x10;
		break;
	case PCD_TRANSCEIVE:
		irqEn = 0x77;
		waitIRq = 0x30;
		break;
	}

	Write_MFRC522(CommIEnReg, irqEn | 0x80); // Omogućavamo prekide
	ClearBitMask(CommIrqReg, 0x80);        // Brišemo zastavice prekida
	SetBitMask(FIFOLevelReg, 0x80);        // Reset FIFO buffera

	Write_MFRC522(CommandReg, PCD_IDLE);   // Prekid trenutne operacije

	/* Slanje podataka u FIFO */
	for (i = 0; i < sendLen; i++)
		Write_MFRC522(FIFODataReg, sendData[i]);

	/* Pokretanje naredbe */
	Write_MFRC522(CommandReg, command);
	if (command == PCD_TRANSCEIVE)
		SetBitMask(BitFramingReg, 0x80); // StartSend = 1

	/* Čekanje završetka operacije */
	i = 2000; // Timeout - prilagoditi ako je RC522 spor
	do {
		n = Read_MFRC522(CommIrqReg);
		i--;
	} while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

	ClearBitMask(BitFramingReg, 0x80); // Stop slanja

	/* Provjera rezultata */
	if (i != 0) {
		if (!(Read_MFRC522(ErrorReg) & 0x1B)) // Nema grešaka
		{
			status = MI_OK;
			if (n & irqEn & 0x01)
				status = MI_NOTAGERR;

			if (command == PCD_TRANSCEIVE) {
				n = Read_MFRC522(FIFOLevelReg);
				lastBits = Read_MFRC522(ControlReg) & 0x07;
				*backLen = lastBits ? (n - 1) * 8 + lastBits : n * 8;
				if (n == 0)
					n = 1;
				if (n > MAX_LEN)
					n = MAX_LEN;

				for (i = 0; i < n; i++)
					backData[i] = Read_MFRC522(FIFODataReg);
			}
		} else
			status = MI_ERR;
	}

	return status;
}

/*
 * Naziv funkcije: MFRC522_Request
 * Opis: Traži prisutnu karticu i čita njen tip (ATQA vrijednost).
 * Ulaz: reqMode - tip zahtjeva (PICC_REQIDL = traži karticu u mirovanju,
 *                  PICC_REQALL = traži sve kartice u RF polju)
 *       TagType - polje u koje se sprema tip kartice (2 bajta)
 * Povrat: MI_OK ako je kartica pronađena i tip pročitan; MI_ERR inače.
 * Napomena: Ovo je prva funkcija u procesu detekcije kartice.
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType) {
	uchar status;
	uint backBits; // broj bitova vraćenih iz odgovora

	Write_MFRC522(BitFramingReg, 0x07); // postavljanje broja validnih bitova u posljednjem bajtu (7 bitova)

	TagType[0] = reqMode; // spremamo tip zahtjeva u prvi bajt
	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	// Ako odgovor nije MI_OK ili ako nema očekivanih 2 bajta (16 bitova)
	if ((status != MI_OK) || (backBits != 0x10))
		status = MI_ERR;

	return status;
}

/*
 * Naziv funkcije: MFRC522_Anticoll
 * Opis: Očitava serijski broj kartice koristeći anti-kolizijski postupak.
 * Ulaz: serNum - polje u koje se sprema serijski broj (5 bajtova, zadnji je kontrolni XOR).
 * Povrat: MI_OK ako je serijski broj uspješno očitan; MI_ERR inače.
 * Napomena: Sprječava kolizije kada je više kartica u RF polju.
 */
uchar MFRC522_Anticoll(uchar *serNum) {
	uchar status;
	uchar i;
	uchar serNumCheck = 0; // kontrolni zbroj XOR svih bajtova
	uint unLen;

	Write_MFRC522(BitFramingReg, 0x00); // bez dodatnog poravnanja bitova

	serNum[0] = PICC_ANTICOLL; // naredba za anti-koliziju
	serNum[1] = 0x20; // NVB - broj bajtova koji se šalju i primaju
	status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	if (status == MI_OK) {
		// Računanje XOR svih 4 bajta serijskog broja
		for (i = 0; i < 4; i++)
			serNumCheck ^= serNum[i];

		// Provjera kontrolnog bajta (5. bajt)
		if (serNumCheck != serNum[i])
			status = MI_ERR;
	}

	return status;
}

/*
 * Naziv funkcije: CalulateCRC
 * Opis: Izračun CRC-a pomoću ugrađenog CRC modula u MFRC522.
 * Ulaz: pIndata - pokazivač na ulazne podatke
 *       len - duljina podataka
 *       pOutData - polje gdje se spremaju 2 bajta CRC rezultata (LSB prvi).
 * Napomena: Umjesto softverskog CRC-a, ovdje se koristi hardverski modul čipa.
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData) {
	uchar i, n;

	ClearBitMask(DivIrqReg, 0x04); // brišemo flag "CRC završeno"
	SetBitMask(FIFOLevelReg, 0x80); // reset FIFO buffera

	// Učitavamo podatke u FIFO
	for (i = 0; i < len; i++)
		Write_MFRC522(FIFODataReg, *(pIndata + i));

	Write_MFRC522(CommandReg, PCD_CALCCRC); // pokretanje CRC izračuna

	i = 0xFF; // timeout brojač (sprječava beskonačnu petlju)
	do {
		n = Read_MFRC522(DivIrqReg); // čitamo status
		i--;
	} while ((i != 0) && !(n & 0x04)); // čekamo bit "CRC završeno"

	// Spremamo izračunati CRC (LSB i MSB)
	pOutData[0] = Read_MFRC522(CRCResultRegL);
	pOutData[1] = Read_MFRC522(CRCResultRegH);
}

/*
 * Naziv funkcije: MFRC522_SelectTag
 * Opis: Odabire određenu karticu prema serijskom broju i čita njen memorijski kapacitet.
 * Ulaz: serNum - polje sa serijskim brojem kartice
 * Povrat: veličina memorije kartice u bajtovima ili 0 ako je greška.
 * Napomena: Ovo je obavezno prije čitanja ili pisanja podataka.
 */
uchar MFRC522_SelectTag(uchar *serNum) {
	uchar i;
	uchar status;
	uchar size;
	uint recvBits;
	uchar buffer[9];

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70; // NVB za SELECT naredbu
	for (i = 0; i < 5; i++) {
		buffer[i + 2] = *(serNum + i);
	}

	CalulateCRC(buffer, 7, &buffer[7]); // dodaj CRC na kraj
	status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	// Ako je odgovor ispravan, buffer[0] sadrži veličinu memorije
	if ((status == MI_OK) && (recvBits == 0x18)) // 24 bita odgovora
		size = buffer[0];
	else
		size = 0;

	return size;
}

/*
 * Naziv funkcije: MFRC522_Auth
 * Opis: Provjerava lozinku i autentificira se za pristup određenom bloku kartice.
 * Ulaz: authMode - tip autentifikacije (PICC_AUTHENT1A ili PICC_AUTHENT1B)
 *       BlockAddr - adresa bloka
 *       Sectorkey - 6-bajtni ključ
 *       serNum - serijski broj kartice
 * Povrat: MI_OK ako je autentifikacija uspjela; MI_ERR inače.
 * Napomena: Autentifikacija se mora obaviti prije čitanja/pisanja.
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey,
		uchar *serNum) {
	uchar status;
	uint recvBits;
	uchar i;
	uchar buff[12];

	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++)
		buff[i + 2] = *(Sectorkey + i);

	for (i = 0; i < 4; i++)
		buff[i + 8] = *(serNum + i);

	status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

	// Provjera je li autentifikacija prošla
	if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08))) // 0x08 = AUTH_OK bit
		status = MI_ERR;

	return status;
}

/*
 * Naziv funkcije: MFRC522_Read
 * Opis: Čita sadržaj jednog bloka podataka s kartice.
 * Ulaz: blockAddr - adresa bloka (0-63 za MIFARE 1K)
 *       recvData - polje gdje se spremaju podaci
 * Povrat: MI_OK ako je čitanje uspjelo; MI_ERR inače.
 * Napomena: Blok ima 16 bajtova; potrebno je prethodno izvršiti autentifikaciju.
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData) {
	uchar status;
	uint unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	CalulateCRC(recvData, 2, &recvData[2]); // dodaj CRC
	status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

	// Provjera duljine odgovora (16 bajtova + CRC = 18 bajtova = 144 bita)
	if ((status != MI_OK) || (unLen != 0x90))
		status = MI_ERR;

	return status;
}

/*
 * Naziv funkcije: MFRC522_Write
 * Opis: Upisuje 16 bajtova podataka u određeni blok kartice.
 * Ulaz: blockAddr - adresa bloka
 *       writeData - podaci za upis (16 bajtova)
 * Povrat: MI_OK ako je upis uspješan; MI_ERR inače.
 * Napomena: Potrebna prethodna autentifikacija.
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData) {
	uchar status;
	uint recvBits;
	uchar i;
	uchar buff[18];

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	CalulateCRC(buff, 2, &buff[2]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

	// Provjera ACK odgovora
	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
		status = MI_ERR;

	if (status == MI_OK) {
		for (i = 0; i < 16; i++)
			buff[i] = *(writeData + i);

		CalulateCRC(buff, 16, &buff[16]); // dodaj CRC
		status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		// Provjera ACK nakon upisa
		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
			status = MI_ERR;
	}

	return status;
}

/*
 * Naziv funkcije: MFRC522_Halt
 * Opis: Prebacuje karticu u stanje mirovanja (HALT naredba).
 * Napomena: Nakon ove naredbe kartica prestaje komunicirati dok se ponovno ne detektira.
 */
void MFRC522_Halt(void) {
	uint unLen;
	uchar buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	CalulateCRC(buff, 2, &buff[2]); // dodaj CRC

	MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen); // slanje HALT naredbe
}


// --- Sensor 1 ---
#define TRIG_PIN GPIO_PIN_9       // TRIG pin za senzor 1
#define TRIG_PORT GPIOA           // Port za TRIG 1

uint32_t IC_Val1 = 0, IC_Val2 = 0;      // Vrijednosti za mjerenje vremena
uint32_t Difference = 0;               // Razlika impulsa
uint8_t Is_First_Captured = 0;         // Zastavica za detekciju ruba
uint8_t Distance = 0;                  // Izračunata udaljenost u cm
// --- Sensor 2 ---
#define TRIG2_PIN GPIO_PIN_1
#define TRIG2_PORT GPIOA
uint32_t IC_Val1_2 = 0, IC_Val2_2 = 0;
uint8_t Is_First_Captured_2 = 0;
uint8_t Distance2 = 0;
uint32_t Difference2 = 0;
// --- Sensor 3 ---
#define TRIG3_PIN GPIO_PIN_4
#define TRIG3_PORT GPIOA
uint32_t IC_Val1_3 = 0, IC_Val2_3 = 0;
uint8_t Is_First_Captured_3 = 0;
uint8_t Distance3 = 0;
uint32_t Difference3 = 0;

// delay za ultrazvucne senzore
void delay(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim6, 0);              // Reset brojača timera
	while (__HAL_TIM_GET_COUNTER(&htim6) < time)
		;  // Čekaj dok ne istekne vrijeme

}

// callback funkcije
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// 1. ULTRAZVUČNI
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // ako je interupt na kanalu 1 aktivan
			{
		if (Is_First_Captured == 0) // Uhvati vrijeme uzlaznog ruba
				{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // pročitaj prvu vrijednost
			Is_First_Captured = 1;  // postavi prvu vrijednost u 1
			// mjenja polaritet na falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING); // Čekaj silazni rub
		}

		else if (Is_First_Captured == 1)   // ako je već u 1
				{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // pročitaj drugu vrijednost
			__HAL_TIM_SET_COUNTER(htim, 0);  // resetiraj brojač

			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034 / 2; // Konverzija u centimetre
			Is_First_Captured = 0; // Reset

			// postavlja polaritet na rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}

	// 2. ULTRAZVUČNI
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (Is_First_Captured_2 == 0) {
			IC_Val1_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			Is_First_Captured_2 = 1;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured_2 == 1) {
			IC_Val2_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if (IC_Val2_2 > IC_Val1_2) {
				Difference2 = IC_Val2_2 - IC_Val1_2;
			}

			else if (IC_Val1_2 > IC_Val2_2) {
				Difference2 = (0xffff - IC_Val1_2) + IC_Val2_2;
			}

			Distance2 = Difference2 * .034 / 2;
			Is_First_Captured_2 = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
		}
	}

	// 3. ULTRAZVUČNI
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		if (Is_First_Captured_3 == 0) {
			IC_Val1_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			Is_First_Captured_3 = 1;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured_3 == 1) {
			IC_Val2_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if (IC_Val2_3 > IC_Val1_3) {
				Difference3 = IC_Val2_3 - IC_Val1_3;
			}

			else if (IC_Val1_3 > IC_Val2_3) {
				Difference3 = (0xffff - IC_Val1_3) + IC_Val2_3;
			}

			Distance3 = Difference3 * .034 / 2;
			Is_First_Captured_3 = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
		}
	}

}

void HCSR04_Read(void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // Pošalji TRIG impuls
	HAL_Delay(1);   // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1); // Omogući interrupt za mjerenje
}
void HCSR04_Read2(void) {
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_SET);
	HAL_Delay(1);   // wait for 10 us
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
}
void HCSR04_Read3(void) {
	HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_SET);
	HAL_Delay(1);   // wait for 10 us
	HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
}

//
void playTone(uint32_t freq, uint32_t duration_ms) {
	uint32_t timer_clk = 1000000;  // After prescaler = 89
	uint32_t period = timer_clk / freq - 1;
	uint32_t duty = (period + 1) / 4;

	__HAL_TIM_SET_AUTORELOAD(&htim3, period);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_Delay(duration_ms);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

// Vrijednosti
uint8_t status;
uint8_t str[16];
uint8_t sNum[4];
typedef enum {
	false = 0, true = 1
} bool;

typedef enum {
	EKRAN_NEPOZNATO,
	EKRAN_KARTICA,
	EKRAN_ZAUZETO,
	EKRAN_ISPRAVNA,
	EKRAN_NEISPRAVNA
} EkranStanje;

EkranStanje trenutnoStanje = EKRAN_NEPOZNATO;

void SystemClock_Config(void);

//postavke za servo motor
int map(int st1, int fn1, int st2, int fn2, int value) {
	return (1.0 * (value - st1)) / ((fn1 - st1) * 1.0) * (fn2 - st2) + st2;
}

void servo_write(int angle) {
	htim2.Instance->CCR1 = map(0, 180, 50, 250, angle); // Pretvori kut u PWM širinu
}

void servo_sweep(void) {
	for (int i = 0; i <= 50; i++) { // Postavljanje kuta na 90 stupnjeva
		servo_write(i);
		HAL_Delay(10);
	}
	HAL_Delay(3000);
	for (int i = 50; i >= 0; i--) { // Postavljanje kuta na 0 stupnjeva
		servo_write(i);
		HAL_Delay(10);
	}

}

void ocitanje(void) { // Funkcija za uČitavanje kartice, provjeru zauzetih mjesta, podizanje rampe i ispisa na ekran

	HCSR04_Read();

	if (Distance <= 5) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	}

	HCSR04_Read2();

	if (Distance2 <= 5) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	}

	HCSR04_Read3();

	if (Distance3 <= 5) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	}

	status = MFRC522_Request(PICC_REQIDL, str);  // Traži RFID tag
	status = MFRC522_Anticoll(str);              // Anti-kolizija (dobije UID)
	memcpy(sNum, str, 4);                        // Kopira UID u buffer

	HAL_Delay(200);

	// Validna kartica
	uint8_t validna_kartica[] = { 0x75, 0xED, 0xE7, 0x2B };
	bool is_valid = true;

	// Provjeri je li očitana kartica validna
	for (int i = 0; i < 4; i++) {
		if (str[i] != validna_kartica[i]) {
			is_valid = false;
			break;
		}
	}

	if (Distance <= 5 && Distance2 <= 5 && Distance3 <= 5
			&& (status != MI_OK || status == MI_OK)) { // Ako su sva mjesta zauzeta
		if (trenutnoStanje != EKRAN_ZAUZETO) {
			oled_Fill(Black);
			oled_SetCursor(0, 0);
			oled_WriteString("Mjesta su zauzeta!", Font_6x8, White);
			oled_UpdateScreen();
			trenutnoStanje = EKRAN_ZAUZETO; // Spremanje varijable
		}
		return;
	}

	if (status != MI_OK) {
		if (trenutnoStanje != EKRAN_KARTICA) { // Defautno stanje
			oled_Fill(Black);
			oled_SetCursor(0, 0);
			oled_WriteString("Prislonite karticu.\n", Font_6x8, White);
			oled_UpdateScreen();
			trenutnoStanje = EKRAN_KARTICA;
		}
		return;
	}

	else if (is_valid) {
		if (trenutnoStanje != EKRAN_ISPRAVNA) { // Ako je kartica ispravna
			HAL_Delay(30);
			playTone(3500, 100);
			HAL_Delay(50);
			playTone(4000, 100);
			HAL_Delay(50);
			playTone(4500, 100);
			HAL_Delay(50);
			oled_Fill(Black);
			oled_SetCursor(0, 0);
			oled_WriteString("Ispravna kartica!", Font_6x8, White);
			oled_UpdateScreen();
			trenutnoStanje = EKRAN_ISPRAVNA;
			servo_sweep();
		}
	}

	else {
		if (trenutnoStanje != EKRAN_NEISPRAVNA) {  // Ako je kartica pogrešna
			HAL_Delay(30);
			playTone(3000, 100);
			HAL_Delay(50);
			playTone(2500, 200);
			HAL_Delay(50);
			oled_Fill(Black);
			oled_SetCursor(0, 0);
			oled_WriteString("Pogresna kartica!", Font_6x8, White);
			oled_UpdateScreen();
			HAL_Delay(3000);
			trenutnoStanje = EKRAN_NEISPRAVNA;
		}
	}

}
/* USER CODE END 0 */

int main(void) {

	HAL_Init();               // Inicijalizacija HAL biblioteke
	SystemClock_Config();     // Konfiguracija takta

	MX_GPIO_Init();           // Inicijalizacija GPIO pinova
	MX_SPI1_Init();           // SPI (za RFID modul)
	MX_USART2_UART_Init();    // UART (ako treba debug)
	MX_I2C1_Init();           // I2C (za OLED)
	MX_TIM1_Init();           // Tajmeri za IC (ultrazvuk)
	MX_TIM6_Init();           // Tajmer za delay()
	MX_TIM3_Init();           // PWM za buzzer
	MX_TIM2_Init();           // PWM za servo motor

	MFRC522_Init();           // Inicijalizacija RFID modula
	oled_Init();           // OLED ekran

	// Start interrupt capture za sva 3 kanala
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Base_Start(&htim6);   // Start delay tajmera
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Start PWM za servo

	while (1) {

		ocitanje(); // Pozivanje glavne funkcije

	}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
