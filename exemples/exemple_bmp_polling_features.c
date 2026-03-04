/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : exemple_bmpx_polling_features.c
  * @brief          : Exemple BMPX — couverture API publique complète
  ******************************************************************************
  *
  * Couvre les fonctions non présentes dans les exemples polling/async :
  *   - BMP_DetectAddress7b()   — détection automatique adresse I2C sur le bus
  *   - BMP_SetAddress()        — configuration adresse arbitraire
  *   - BMP_UsePrimaryAddress() — basculement adresse primaire (0x76)
  *   - BMP_UseSecondaryAddress()— basculement adresse secondaire (0x77)
  *   - BMP_ReadTemperature()   — lecture température seule
  *   - BMP_ReadPressure()      — lecture pression seule
  *   - BMP_Async_Reset()       — remise à zéro contexte async (récupération)
  *   - BMP_Async_SetCallbacks()— callbacks data-ready + erreur (boucle main)
  *   - BMP_Async_SetIrqCallbacks() — callbacks IRQ-safe (contexte interruption)
  *   - BMP_Async_GetData()     — lecture données si flag data_ready levé
  *   - BMP_Async_ClearFlags()  — acquittement manuel drapeaux data/erreur
  *
  * CubeMX requis (Nucleo L476RG) :
  *   - I2C3 : SCL=PC0, SDA=PC1, Standard mode, Global interrupt activée
  *   - USART2 : 115200 bauds (printf)
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
#define LOG_NAME            "exemple_bmpx_polling_features" ///< Nom affiché dans le header terminal
#define MEASURE_INTERVAL_MS     2000u ///< Intervalle entre deux mesures async (ms)
#define MAX_ERRORS              5u    ///< Seuil d'erreurs avant BMP_Async_Reset()
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* @note FreeRTOS : appeler BMP_Async_TriggerEvery() + BMP_Async_Tick() depuis une seule tâche.
 *       Les callbacks OnI2CMasterTxCplt/RxCplt/OnI2CError s'exécutent en IRQ et
 *       n'utilisent que des champs volatile — sûrs sans mutex.
 *       Ne pas appeler BMP_Async_Process() depuis plusieurs tâches simultanément. */
/* USER CODE END PM */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static BMP_Handle_t  hbmp;                       ///< Handle principal capteur BMPX
static BMP_Async   bmp_async;                  ///< Contexte asynchrone BMPX
static volatile bool g_data_ready  = false;   ///< Flag données prêtes (set depuis IRQ)
static volatile bool g_error_flag  = false;   ///< Flag erreur async (set depuis IRQ)
static BMP_Status    g_last_status = BMP_OK;  ///< Dernier statut async
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
/* Callbacks main-loop (appelés depuis BMP_Async_Tick) */
static void OnBmpDataReady(void *user_ctx, const BMP_Data *data, BMP_Status st);
static void OnBmpError(void *user_ctx, BMP_Status st);
/* Callbacks IRQ-safe (appelés depuis les IRQ HAL — volatile uniquement) */
static void OnBmpIrqDataReady(void *user_ctx);
static void OnBmpIrqError(void *user_ctx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Pour Envoyer le caractère via UART
    // ITM_SendChar(ch);                 // Option alternative pour envoyer le caractère via ITM
    return ch;
}

static void OnBmpDataReady(void *user_ctx, const BMP_Data *data, BMP_Status st)
{
  (void)user_ctx;
  if (st == BMP_OK) {
    printf("[CB-DATA] T=%.2f C | P=%.2f hPa\r\n",
           data->temperature,
           (double)data->pressure / 100.0);
  }
}

static void OnBmpError(void *user_ctx, BMP_Status st)
{
  (void)user_ctx;
  printf("[CB-ERR ] %s\r\n", BMP_StatusToString(st));
}

static void OnBmpIrqDataReady(void *user_ctx)
{
  (void)user_ctx;
  g_data_ready = true; /* signal vers main loop — sans printf dans l'IRQ */
}

static void OnBmpIrqError(void *user_ctx)
{
  (void)user_ctx;
  g_error_flag = true;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\n============================================================\r\n");
  printf("  BMPX — test_bmpx_features : couverture API publique\r\n");
  printf("============================================================\r\n\r\n");

  /* ------------------------------------------------------------------ */
  /* 1. BMP_DetectAddress7b : scan automatique sur le bus I2C            */
  /* ------------------------------------------------------------------ */
  printf("[1] BMP_DetectAddress7b...\r\n");
  uint8_t detected_addr = 0;
  BMP_Status st = BMP_DetectAddress7b(&hi2c3, &detected_addr);
  if (st == BMP_OK) {
    printf("    Capteur detecte  : 0x%02X\r\n\r\n", (unsigned)detected_addr);
  } else {
    printf("    Aucun capteur (bus vide ?) : %s — on continue avec 0x76\r\n\r\n",
           BMP_StatusToString(st));
    detected_addr = BMP_I2C_ADDR_PRIMARY_7B;
  }

  /* ------------------------------------------------------------------ */
  /* 2. BMP_InitAuto (init, chip-id) puis test APIs d'adressage          */
  /* ------------------------------------------------------------------ */
  printf("[2] BMP_InitAuto...\r\n");
  st = BMP_InitAuto(&hbmp, &hi2c3, BMP_DEFAULT_MODE);
  if (st != BMP_OK) {
    printf("    ERREUR Init : %s\r\n", BMP_StatusToString(st));
    BMP_DeInit(&hbmp); ///< Libère le handle avant Error_Handler()
    Error_Handler();
  }
  printf("    Sensor : %s  addr : 0x%02X\r\n\r\n",
         BMP_SensorTypeToString(hbmp.sensorType), (unsigned)BMP_GetAddress(&hbmp));

  /* Couverture API: BMP_InitWithAddress */
  st = BMP_InitWithAddress(&hbmp, &hi2c3, BMP_GetAddress(&hbmp), BMP_DEFAULT_MODE);
  printf("    ReInit explicite (InitWithAddress): %s\r\n\r\n", BMP_StatusToString(st));

  /* BMP_SetAddress — forcer une adresse arbitraire */
  printf("[3] BMP_SetAddress(0x77)...\r\n");
  st = BMP_SetAddress(&hbmp, BMP_I2C_ADDR_SECONDARY_7B);
  printf("    %s\r\n\r\n", BMP_StatusToString(st));

  /* BMP_UsePrimaryAddress / BMP_UseSecondaryAddress */
  printf("[4] BMP_UsePrimaryAddress...\r\n");
  st = BMP_UsePrimaryAddress(&hbmp);
  printf("    addr active : 0x%02X  status : %s\r\n\r\n",
         (unsigned)BMP_GetAddress(&hbmp), BMP_StatusToString(st));

  printf("[5] BMP_UseSecondaryAddress...\r\n");
  st = BMP_UseSecondaryAddress(&hbmp);
  printf("    addr active : 0x%02X  status : %s\r\n\r\n",
         (unsigned)BMP_GetAddress(&hbmp), BMP_StatusToString(st));

  /* Remettre en primaire pour les mesures */
  BMP_UsePrimaryAddress(&hbmp);

  /* ------------------------------------------------------------------ */
  /* 6. BMP_ReadTemperature / BMP_ReadPressure — lectures individuelles  */
  /* ------------------------------------------------------------------ */
  printf("[6] BMP_ReadTemperature + BMP_ReadPressure (lectures individuelles)...\r\n");
  float temp_c    = 0.0f;
  int32_t pres_pa = 0;
  st = BMP_ReadTemperature(&hbmp, &temp_c);
  printf("    T = %.2f C  (%s)\r\n", temp_c, BMP_StatusToString(st));
  st = BMP_ReadPressure(&hbmp, &pres_pa);
  printf("    P = %ld Pa  (%s)\r\n\r\n", (long)pres_pa, BMP_StatusToString(st));

  /* ------------------------------------------------------------------ */
  /* 7. BMP_Async_Init puis Reset (remet l'état machine à 0)             */
  /* ------------------------------------------------------------------ */
  printf("[7] BMP_Async_Init + BMP_Async_Reset...\r\n");
  BMP_Async_Init(&bmp_async, &hbmp);
  /* Configurer l'intervalle si différent du défaut (BMPX_DEFAULT_SAMPLE_INTERVAL_MS=2000ms) */
  bmp_async.sample_interval_ms = MEASURE_INTERVAL_MS;
  printf("    Contexte async prêt\r\n\r\n");

  /* ------------------------------------------------------------------ */
  /* 8. BMP_Async_SetCallbacks — callbacks main-loop (data-ready/erreur) */
  /* ------------------------------------------------------------------ */
  printf("[8] BMP_Async_SetCallbacks...\r\n");
  BMP_Async_SetCallbacks(&bmp_async, OnBmpDataReady, OnBmpError, NULL);
  printf("    Callbacks data-ready + erreur enregistres\r\n\r\n");

  /* ------------------------------------------------------------------ */
  /* 9. BMP_Async_SetIrqCallbacks — callbacks IRQ-safe                   */
  /* ------------------------------------------------------------------ */
  printf("[9] BMP_Async_SetIrqCallbacks...\r\n");
  BMP_Async_SetIrqCallbacks(&bmp_async, OnBmpIrqDataReady, OnBmpIrqError, NULL);
  printf("    Callbacks IRQ-safe enregistres\r\n\r\n");

  /* Déclenchement initial */
  st = BMP_ReadAll_IT(&bmp_async);
  if (st != BMP_OK) {
    printf("ERREUR  config I2C/IRQ incomplète : %s\r\n", BMP_StatusToString(st));
    printf("Action : .ioc -> I2Cx -> NVIC -> Global interrupt activee\r\n");
    Error_Handler();
  }

  uint32_t last_trigger = HAL_GetTick();
  uint32_t error_count  = 0;

  printf("Boucle principale — appuyez Reset pour arrêter\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* ----- BMP_Async_Tick : avance la machine d'état ----- */
    BMP_Data sample;
    BMP_TickResult tick = BMP_Async_Tick(&bmp_async, now, &sample);

    if (tick == BMP_TICK_DATA_READY) {
      printf("[TICK] T=%.2f C | P=%.0f Pa \r\n",
             sample.temperature,
             (double)sample.pressure / 100.0);
      error_count = 0;

      /* Couverture API: BMP_CalculateSeaLevelPressure */
      int32_t p0_pa = BMP_CalculateSeaLevelPressure(sample.pressure, 150.0f);
      printf("[P0]  P0(150m)=%.0f hPa\r\n", (double)p0_pa / 100.0);

      /* ---------------------------------------------------------------- */
      /* 10. BMP_Async_GetData — lecture données si flag levé             */
      /* ---------------------------------------------------------------- */
      (void)BMP_Async_DataReadyFlag(&bmp_async);
      (void)BMP_Async_ErrorFlag(&bmp_async);
      (void)BMP_Async_HasData(&bmp_async);
      BMP_Data out;
      if (BMP_Async_GetData(&bmp_async, &out) == BMP_OK) {
        printf("[GETDATA] T=%.2f C | P=%.0f Pa\r\n",
               out.temperature, (double)out.pressure / 100.0);
      }

      /* ---------------------------------------------------------------- */
      /* 11. BMP_Async_ClearFlags — acquittement manuel des drapeaux      */
      /* ---------------------------------------------------------------- */
      BMP_Async_ClearFlags(&bmp_async);

      last_trigger = now;
    }
    else if (tick == BMP_TICK_ERROR) {
      error_count++;
      printf("[ERR] %s (total=%lu)\r\n",
             BMP_StatusToString(bmp_async.last_status),
             (unsigned long)error_count);
      if (error_count >= MAX_ERRORS) {
        printf("[RESET] trop d'erreurs — BMP_Async_Reset + ré-init\r\n");
        BMP_Async_Reset(&bmp_async);
        BMP_Async_Init(&bmp_async, &hbmp);
        BMP_Async_SetCallbacks(&bmp_async, OnBmpDataReady, OnBmpError, NULL);
        BMP_Async_SetIrqCallbacks(&bmp_async, OnBmpIrqDataReady, OnBmpIrqError, NULL);
        error_count = 0;
      }
    }

    /* ----- Déclenchement périodique via TriggerEvery (3 params) ----- */
    BMP_Async_TriggerEvery(&bmp_async, now, &last_trigger);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BMP_Async_OnI2CMasterTxCplt(&bmp_async, hi2c);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BMP_Async_OnI2CMasterRxCplt(&bmp_async, hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  BMP_Async_OnI2CError(&bmp_async, hi2c);
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
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

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
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
  }}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
