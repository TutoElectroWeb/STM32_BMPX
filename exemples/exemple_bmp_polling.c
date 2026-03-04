/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : exemple_bmpx_polling.c
  * @brief          : Exemple BMPX polling avec auto-détection capteur
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "STM32_BMPX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME                    "exemple_bmpx_polling" ///< Nom affiché dans le header terminal
#define BMPX_RECOVERY_ERR_THRESHOLD 2u ///< Nombre d'erreurs I2C consécutives avant recovery (re-init)
#define MAX_CONSECUTIVE_ERRORS      BMPX_RECOVERY_ERR_THRESHOLD ///< Alias — seuil d'erreurs consécutives avant arrêt
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static BMP_Handle_t hbmp; ///< Handle principal du capteur BMPX (BMP085/180/BMP280)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Pour Envoyer le caractère via UART
    // ITM_SendChar(ch);                 // Option alternative pour envoyer le caractère via ITM
    return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
	HAL_Delay(1000);
  printf("\r\n========================================\r\n");
  printf("  Fichier: exemple_bmpx_polling\r\n");
  printf("  BMPX - Polling auto-detect\r\n");
  printf("========================================\r\n\r\n");

  BMP_Status st_init = BMP_InitAuto(&hbmp, &hi2c3, BMP_ULTRAHIGHRES);
  if (st_init != BMP_OK)
  {
    printf("ERREUR  INIT: aucun capteur detecte sur 0x76/0x77 (%s)\r\n", BMP_StatusToString(st_init));
    BMP_DeInit(&hbmp); ///< Libère le handle avant Error_Handler()
    Error_Handler();
  }
  printf("INFO  Initialisation BMPX (addr 0x%02X) ...\r\n", BMP_GetAddress(&hbmp));
  printf("OK  Capteur detecte: %s\r\n", BMP_SensorTypeToString(hbmp.sensorType));

  uint32_t comm_error_count = 0u;

  if (hbmp.sensorType == BMP_SENSOR_BMP280)
	{
    BMP280_Config cfg;
    cfg.osrs_t = BMP280_OSRS_X1;
    cfg.osrs_p = BMP280_OSRS_X16;
    cfg.filter = BMP280_FILTER_OFF;
    cfg.standby = BMP280_STANDBY_0P5_MS;
    cfg.mode = BMP280_MODE_SLEEP;

    BMP_Status st_cfg = BMP280_SetConfig(&hbmp, &cfg);
    if (st_cfg != BMP_OK)
    {
      printf("ERREUR  CFG BMP280: %s (%d)\r\n", BMP_StatusToString(st_cfg), (int)st_cfg);
      Error_Handler();
    }
    printf("INFO  Mode BMP280 oneshot FORCED actif\r\n\r\n");
	}
  else
  {
    printf("INFO  Mode BMP085/BMP180 polling actif\r\n\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		float temperature_degC = 0.0f;
		int32_t pressure_Pa = 0;

    BMP_Status st;
    if (hbmp.sensorType == BMP_SENSOR_BMP280)
    {
      st = BMP280_ReadAll_OneShot(&hbmp, &temperature_degC, &pressure_Pa, 200);
    }
    else
    {
      st = BMP_ReadAll(&hbmp, &temperature_degC, &pressure_Pa);
    }

		if (st != BMP_OK)
		{
      printf("ERREUR  MESURE: %s (%d)\r\n", BMP_StatusToString(st), (int)st);
      comm_error_count++;
      if (comm_error_count >= BMPX_RECOVERY_ERR_THRESHOLD) {
        printf("INFO  Recovery BMPX apres erreurs I2C consecutives...\r\n");
        st_init = BMP_InitAuto(&hbmp, &hi2c3, BMP_ULTRAHIGHRES);
        if (st_init == BMP_OK) {
          comm_error_count = 0u;
          printf("OK  Recovery BMPX (addr 0x%02X, %s)\r\n", BMP_GetAddress(&hbmp), BMP_SensorTypeToString(hbmp.sensorType));
        } else {
          printf("ERREUR  Recovery BMPX impossible (%s)\r\n", BMP_StatusToString(st_init));
        }
      }
      HAL_Delay(500);
      continue;
		}

    comm_error_count = 0u;

    printf("OK  %s - T: %.2f C  P: %ld Pa\r\n",
           BMP_SensorTypeToString(hbmp.sensorType), temperature_degC, (long)pressure_Pa);
		HAL_Delay(2000);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);                      // Fait clignoter la LED d'erreur
      for (volatile uint32_t wait = 0U; wait < 250000U; ++wait) { __NOP(); } // Temporisation 250 ms sans HAL_Delay
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
