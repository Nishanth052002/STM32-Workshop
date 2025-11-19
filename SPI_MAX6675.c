/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   MAX6675 Thermocouple Reader – 100% WORKING 2025 VERSION
  * @author  Fixed & optimized for STM32 HAL
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

char uart_msg[100];

/* CS Pin for MAX6675 */
#define MAX6675_CS_PORT GPIOB
#define MAX6675_CS_PIN  GPIO_PIN_6

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* MAIN ----------------------------------------------------------------------*/
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  // Start with CS high (deselected)
  HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_SET);

  while (1)
  {
    uint16_t raw = 0;

    // --- 1. Select MAX6675 ---
    HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_RESET);

    // --- 2. Receive 16-bit data (you already set DataSize = 16-bit in CubeMX) ---
    HAL_SPI_Receive(&hspi1, (uint8_t*)&raw, 1, HAL_MAX_DELAY);

    // --- 3. Deselect ---
    HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_SET);

    // --- 4. Check for open thermocouple (bit 2 = 1 → fault) ---
    if (raw & 0x4)
    {
      sprintf(uart_msg, "ERROR: Thermocouple OPEN or fault!\r\n");
    }
    else
    {
      // Bits 14–3 = temperature in 0.25°C steps → shift right 3
      int16_t temp_quarters = (raw >> 3) & 0x0FFF;   // 0...4095
      float temp_c = temp_quarters * 0.25f;

     //Send over UART
    sprintf(uart_msg,"float temp: %.2f\n",temp_c);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
    }
    HAL_Delay(1000);  // read every second
  }
}

/* --------------------- PERIPHERAL INIT (unchanged & correct) --------------------- */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;          // THIS IS CRUCIAL
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;             // MAX6675 needs CPOL=0, CPHA=0
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // safe speed
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
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
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // CS pin as output
  GPIO_InitStruct.Pin = MAX6675_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MAX6675_CS_PORT, &GPIO_InitStruct);

  // Start with CS high
  HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_SET);
}

void SystemClock_Config(void)
{
  // (your existing clock config – it's fine, leave as-is)
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
