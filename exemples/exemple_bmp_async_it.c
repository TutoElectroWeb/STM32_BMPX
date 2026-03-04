/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : exemple_bmpx_async_it.c
  * @brief          : Exemple BMPX async non-bloquant (I2C IT)
  ******************************************************************************
  * Pré-requis CubeMX:
  * - I2C3 activé (I2C, addressing 7-bit) + broches SCL/SDA configurées par CubeMX
  *   (pull-ups externes requis sur le bus I2C, typiquement 4.7 kΩ)
  * - NVIC: activer "I2C3 event interrupt" et "I2C3 error interrupt"
  * - USART2 optionnel (Asynchronous, 115200 8N1) si tu utilises printf via __io_putchar()
  *
  * Notes:
  * - Fichier d'exemple (non compilé par défaut)
  * - La mesure est déclenchée toutes les 2 s.
  * - La boucle principale ne bloque pas: progression via callbacks I2C + BMP_Async_Process().
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "STM32_BMPX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_bmpx_async_it" ///< Nom du fichier affiché dans le header terminal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* @note FreeRTOS : appeler BMP_Async_TriggerEvery() + BMP_Async_Tick() depuis une seule tâche
 *       (ex: Task_Sensors). Les callbacks OnI2CMasterTxCplt/RxCplt/OnI2CError s'exécutent
 *       dans le contexte IRQ et n'utilisent que des champs volatile — sûrs sans mutex.
 *       Ne pas appeler BMP_Async_Process() depuis plusieurs tâches simultanément. */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BMP_Handle_t bmp;        ///< Handle principal capteur BMPX
BMP_Async_t bmp_async;   ///< Contexte asynchrone BMPX (FSM IT)
static uint8_t preflight_error_count = 0; ///< Compteur erreurs avant première mesure valide
static bool first_valid_sample = false;   ///< true après la première mesure OK
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
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

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  BMP_Status status;
  uint32_t last_trigger_time = 0;
  uint32_t measure_count = 0;

  printf("\r\n========================================\r\n");
  printf("  Fichier: " LOG_NAME "\r\n");
  printf("  BMPX - Async IT (non-bloquant)\r\n");
  printf("========================================\r\n\r\n");

  BMP_Status st_init = BMP_InitAuto(&bmp, &hi2c3, BMP_DEFAULT_MODE);
  if (st_init != BMP_OK) {
    printf("ERREUR  Init: aucun capteur detecte sur 0x76/0x77 (%s)\r\n", BMP_StatusToString(st_init));
    BMP_DeInit(&bmp); ///< Libère le handle avant Error_Handler()
    Error_Handler();
  }

  printf("OK  %s detecte (addr 0x%02X)\r\n", BMP_SensorTypeToString(bmp.sensorType), BMP_GetAddress(&bmp));

  BMP_Async_Init(&bmp_async, &bmp);
  /* Intervalle de mesure : utilise BMPX_DEFAULT_SAMPLE_INTERVAL_MS (2000 ms) par défaut.
   * Pour changer : bmp_async.sample_interval_ms = 5000U; */

  printf("Mode : Interruptions I2C (IT)\r\n");
  printf("   Mesure toutes les 2 secondes\r\n");
  printf("   Callbacks HAL I2C inclus dans USER CODE BEGIN 4\r\n");
  printf("   Validation MX (I2C/IRQ) effectuée par la librairie\r\n\r\n");

  /* Premier déclenchement — valide immédiatement la config I2C/IRQ */
  BMP_Status trig_init = BMP_ReadAll_IT(&bmp_async);
  if (trig_init != BMP_OK) {
    printf("ERREUR  Configuration MX incomplète (I2C/IRQ) pour mode IT\r\n");
    printf("   Action : .ioc -> Connectivity -> I2Cx -> Configuration -> NVIC Settings\r\n");
    printf("            cocher I2C event interrupt + I2C error interrupt\r\n");
    printf("   Code   : %s\r\n", BMP_StatusToString(trig_init));
    Error_Handler();
  }
  last_trigger_time = HAL_GetTick();
  printf("INFO  Première mesure lancée...\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    BMP_Data sample;

    /* Déclenchement périodique non-bloquant via TriggerEvery (3 params) */
    if (BMP_Async_IsIdle(&bmp_async)) {
      BMP_Async_TriggerEvery(&bmp_async, now, &last_trigger_time);
    }

    BMP_TickResult tick = BMP_Async_Tick(&bmp_async, now, &sample);
    if (tick == BMP_TICK_DATA_READY) {
      first_valid_sample = true;
      preflight_error_count = 0U;
      float altitude = 0.0f;
      BMP_CalculateAltitude(sample.pressure, 101325.0f, &altitude);
      measure_count++;
      printf("[#%lu] OK  T=%.2f C | P=%ld Pa (%.2f hPa) | Alt=%.1f m\r\n",
             measure_count, sample.temperature,
             sample.pressure, sample.pressure / 100.0f, altitude);
    } else if (tick == BMP_TICK_ERROR) {
      BMP_Status err = bmp_async.last_status;
      if (!first_valid_sample) {
        preflight_error_count++;
        if (preflight_error_count >= 2U) {
          printf("ERREUR  Configuration MX incomplète (I2C/IRQ) détectée\r\n");
          printf("   Action : .ioc -> Connectivity -> I2Cx -> Configuration -> NVIC Settings\r\n");
          printf("            cocher I2C event interrupt + I2C error interrupt\r\n");
          printf("   Dernier code : %s\r\n", BMP_StatusToString(err));
          Error_Handler();
        }
      }
      if (first_valid_sample) {
        printf("ERREUR  Erreur async: %s -> reset\r\n", BMP_StatusToString(err));
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* USER CODE BEGIN SystemClock_Config 0 */

  /* USER CODE END SystemClock_Config 0 */

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  /* USER CODE BEGIN SystemClock_Config 1 */

  /* USER CODE END SystemClock_Config 1 */
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Callback HAL I2C fin d'emission memoire (IT).
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == bmp.hi2c) {
    BMP_Async_OnI2CMasterTxCplt(&bmp_async, hi2c);
  }
}

/**
 * @brief  Callback HAL I2C fin de reception memoire (IT).
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == bmp.hi2c) {
    BMP_Async_OnI2CMasterRxCplt(&bmp_async, hi2c);
  }
}

/**
 * @brief  Callback HAL I2C erreur (IT).
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == bmp.hi2c) {
    BMP_Async_OnI2CError(&bmp_async, hi2c);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
