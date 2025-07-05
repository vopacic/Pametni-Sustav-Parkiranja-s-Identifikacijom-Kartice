#include "main.h"             // Glavna zaglavlja generirana od STM32CubeMX
#include "i2c.h"              // Inicijalizacija I2C periferije
#include "spi.h"              // Inicijalizacija SPI periferije
#include "tim.h"              // Tajmeri
#include "usart.h"            // Serijska komunikacija
#include "gpio.h"             // Rad s pinovima (input/output)

#include "RC522.h"            // RFID biblioteka
#include "string.h"           // Standardna string biblioteka (npr. memcpy)
#include "ssd1306.h"          // OLED ekran kontrola
#include "ssd1306_fonts.h"    // Fontovi za OLED ekran

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
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("Mjesta su zauzeta!", Font_6x8, White);
			ssd1306_UpdateScreen();
			trenutnoStanje = EKRAN_ZAUZETO; // Spremanje varijable
		}
		return;
	}

	if (status != MI_OK) {
		if (trenutnoStanje != EKRAN_KARTICA) { // Defautno stanje
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("Prislonite karticu.\n", Font_6x8, White);
			ssd1306_UpdateScreen();
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
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("Ispravna kartica!", Font_6x8, White);
			ssd1306_UpdateScreen();
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
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("Pogresna kartica!", Font_6x8, White);
			ssd1306_UpdateScreen();
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
	ssd1306_Init();           // OLED ekran

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
