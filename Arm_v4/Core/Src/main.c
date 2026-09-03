/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static float s1_real = 0.0f, s1_i = 0.0f, s1_j = 0.0f, s1_k = 0.0f;
static float s2_real = 0.0f, s2_i = 0.0f, s2_j = 0.0f, s2_k = 0.0f;

/* TEMP DEBUG counters, remove once BLE data flow is confirmed working */
static volatile uint32_t dbg_s1_events = 0;
static volatile uint32_t dbg_s1_grv    = 0;
static volatile uint32_t dbg_s2_pkts   = 0;
static volatile uint32_t dbg_s2_grv    = 0;
static int dbg_s1_cfg_ret  = -99;
static int dbg_s2_send_ret = -99;
static volatile uint32_t dbg_int1_low = 0;
static volatile uint32_t dbg_int2_low = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static int  spihal_open(sh2_Hal_t *self);
static void spihal_close(sh2_Hal_t *self);
static int  spihal_wake(void);
static int  spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int  spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static uint32_t spihal_getTimeUs(sh2_Hal_t *self);
static sh2_Hal_t *sh2_hal_init(void);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event);
static void BNO08x_Init(void);
static void sh2_service_wrapper(void);

static int  spihal2_wake(void);
static int  spihal2_reset(void);
static uint16_t spihal2_read(uint8_t *pBuffer, uint16_t len);
static int  spihal2_write(uint8_t *pBuffer, uint16_t len);
static int  s2_send(uint8_t chan, const uint8_t *payload, uint16_t len);
static void s2_handlePacket(uint8_t *buf, uint16_t totalLen);
static void BNO08x2_Init(void);
static void BNO08x2_Service(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int spihal_open(sh2_Hal_t *self)
{
    HAL_GPIO_WritePin(BNO_CS1_GPIO_Port, BNO_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BNO_WAKE1_GPIO_Port, BNO_WAKE1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BNO_RST1_GPIO_Port, BNO_RST1_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BNO_RST1_GPIO_Port, BNO_RST1_Pin, GPIO_PIN_SET);

    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BNO_INT1_GPIO_Port, BNO_INT1_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 500U) {
            return SH2_ERR_IO;
        }
    }
    return SH2_OK;
}

static void spihal_close(sh2_Hal_t *self)
{
    (void)self;
}

static int spihal_wake(void)
{
    if (HAL_GPIO_ReadPin(BNO_INT1_GPIO_Port, BNO_INT1_Pin) == GPIO_PIN_RESET) {
        return 1;
    }

    HAL_GPIO_WritePin(BNO_WAKE1_GPIO_Port, BNO_WAKE1_Pin, GPIO_PIN_RESET);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BNO_INT1_GPIO_Port, BNO_INT1_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 500U) {
            HAL_GPIO_WritePin(BNO_WAKE1_GPIO_Port, BNO_WAKE1_Pin, GPIO_PIN_SET);
            return 0;
        }
    }
    HAL_GPIO_WritePin(BNO_WAKE1_GPIO_Port, BNO_WAKE1_Pin, GPIO_PIN_SET);
    return 1;
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;

    if (HAL_GPIO_ReadPin(BNO_INT1_GPIO_Port, BNO_INT1_Pin) == GPIO_PIN_SET) {
        return 0;
    }

    uint8_t header[4] = {0};
    uint8_t dummy[4] = {0};

    HAL_GPIO_WritePin(BNO_CS1_GPIO_Port, BNO_CS1_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, dummy, header, 4, 100);

    uint16_t packetLen = ((header[1] << 8) | header[0]) & 0x7FFFU;
    if (packetLen == 0U || packetLen > len) {
        HAL_GPIO_WritePin(BNO_CS1_GPIO_Port, BNO_CS1_Pin, GPIO_PIN_SET);
        return 0;
    }

    memcpy(pBuffer, header, 4U);
    uint16_t remain = (uint16_t)(packetLen - 4U);
    if (remain > 0U) {
        uint8_t *dummy2 = (uint8_t *)malloc(remain);
        if (dummy2 == NULL) {
            HAL_GPIO_WritePin(BNO_CS1_GPIO_Port, BNO_CS1_Pin, GPIO_PIN_SET);
            return 0;
        }
        memset(dummy2, 0, remain);
        HAL_SPI_TransmitReceive(&hspi1, dummy2, pBuffer + 4U, remain, 100);
        free(dummy2);
    }
    HAL_GPIO_WritePin(BNO_CS1_GPIO_Port, BNO_CS1_Pin, GPIO_PIN_SET);

    if (t_us != NULL) {
        *t_us = HAL_GetTick() * 1000U;
    }
    return (int)packetLen;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;
    if (!spihal_wake()) {
        return 0;
    }

    HAL_GPIO_WritePin(BNO_CS1_GPIO_Port, BNO_CS1_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pBuffer, len, 100);
    HAL_GPIO_WritePin(BNO_CS1_GPIO_Port, BNO_CS1_Pin, GPIO_PIN_SET);
    return (int)len;
}

static uint32_t spihal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return HAL_GetTick() * 1000U;
}

static sh2_Hal_t sh2Hal =
{
    .open = spihal_open,
    .close = spihal_close,
    .read = spihal_read,
    .write = spihal_write,
    .getTimeUs = spihal_getTimeUs,
};

static sh2_Hal_t *sh2_hal_init(void)
{
    return &sh2Hal;
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        return;
    }
    dbg_s1_events++;

    if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
        dbg_s1_grv++;
        s1_real = value.un.gameRotationVector.real;
        s1_i    = value.un.gameRotationVector.i;
        s1_j    = value.un.gameRotationVector.j;
        s1_k    = value.un.gameRotationVector.k;
    }
}

#define S2_CHAN_EXECUTABLE_DEVICE (1)
#define S2_CHAN_SENSORHUB_CONTROL (2)
#define S2_CHAN_SENSORHUB_INPUT   (3)
#define S2_RESET_COMPLETE         (1)
#define S2_SET_FEATURE_CMD        (0xFD)
#define S2_GAME_ROTATION_VECTOR   (0x08)
#define S2_BASE_TIMESTAMP_REF     (0xFB)
#define S2_TIMESTAMP_REBASE       (0xFA)

static uint8_t s2_outSeq[8] = {0};
static bool s2_resetComplete = false;

static int spihal2_wake(void)
{
    if (HAL_GPIO_ReadPin(BNO_INT2_GPIO_Port, BNO_INT2_Pin) == GPIO_PIN_RESET) {
        return 1;
    }

    HAL_GPIO_WritePin(BNO_WAKE2_GPIO_Port, BNO_WAKE2_Pin, GPIO_PIN_RESET);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BNO_INT2_GPIO_Port, BNO_INT2_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 500U) {
            HAL_GPIO_WritePin(BNO_WAKE2_GPIO_Port, BNO_WAKE2_Pin, GPIO_PIN_SET);
            return 0;
        }
    }
    HAL_GPIO_WritePin(BNO_WAKE2_GPIO_Port, BNO_WAKE2_Pin, GPIO_PIN_SET);
    return 1;
}

static int spihal2_reset(void)
{
    HAL_GPIO_WritePin(BNO_CS2_GPIO_Port, BNO_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BNO_WAKE2_GPIO_Port, BNO_WAKE2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BNO_RST2_GPIO_Port, BNO_RST2_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(BNO_RST2_GPIO_Port, BNO_RST2_Pin, GPIO_PIN_SET);

    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BNO_INT2_GPIO_Port, BNO_INT2_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 500U) {
            return 0;
        }
    }
    return 1;
}

static uint16_t spihal2_read(uint8_t *pBuffer, uint16_t len)
{
    if (HAL_GPIO_ReadPin(BNO_INT2_GPIO_Port, BNO_INT2_Pin) == GPIO_PIN_SET) {
        return 0;
    }

    uint8_t header[4] = {0};
    uint8_t dummy[4] = {0};

    HAL_GPIO_WritePin(BNO_CS2_GPIO_Port, BNO_CS2_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, dummy, header, 4, 100);

    uint16_t packetLen = ((header[1] << 8) | header[0]) & 0x7FFFU;
    if (packetLen == 0U || packetLen > len) {
        HAL_GPIO_WritePin(BNO_CS2_GPIO_Port, BNO_CS2_Pin, GPIO_PIN_SET);
        return 0;
    }

    memcpy(pBuffer, header, 4U);
    uint16_t remain = (uint16_t)(packetLen - 4U);
    if (remain > 0U) {
        uint8_t *dummy2 = (uint8_t *)malloc(remain);
        if (dummy2 == NULL) {
            HAL_GPIO_WritePin(BNO_CS2_GPIO_Port, BNO_CS2_Pin, GPIO_PIN_SET);
            return 0;
        }
        memset(dummy2, 0, remain);
        HAL_SPI_TransmitReceive(&hspi1, dummy2, pBuffer + 4U, remain, 100);
        free(dummy2);
    }
    HAL_GPIO_WritePin(BNO_CS2_GPIO_Port, BNO_CS2_Pin, GPIO_PIN_SET);
    return packetLen;
}

static int spihal2_write(uint8_t *pBuffer, uint16_t len)
{
    if (!spihal2_wake()) {
        return 0;
    }

    HAL_GPIO_WritePin(BNO_CS2_GPIO_Port, BNO_CS2_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pBuffer, len, 100);
    HAL_GPIO_WritePin(BNO_CS2_GPIO_Port, BNO_CS2_Pin, GPIO_PIN_SET);
    return len;
}

static int s2_send(uint8_t chan, const uint8_t *payload, uint16_t len)
{
    uint8_t txBuf[24] = {0};
    uint16_t total = (uint16_t)(len + 4U);
    txBuf[0] = (uint8_t)(total & 0xFFU);
    txBuf[1] = (uint8_t)((total >> 8) & 0x7FU);
    txBuf[2] = chan;
    txBuf[3] = s2_outSeq[chan]++;
    memcpy(txBuf + 4U, payload, len);
    return spihal2_write(txBuf, total);
}

static void s2_handlePacket(uint8_t *buf, uint16_t totalLen)
{
    uint8_t chan = buf[2];
    uint8_t *payload = buf + 4U;
    uint16_t plen = (uint16_t)(totalLen - 4U);

    if (chan == S2_CHAN_EXECUTABLE_DEVICE) {
        if (plen >= 1U && payload[0] == S2_RESET_COMPLETE) {
            s2_resetComplete = true;
        }
        return;
    }

    if (chan != S2_CHAN_SENSORHUB_INPUT) {
        return;
    }
    dbg_s2_pkts++;

    uint16_t cursor = 0U;
    while (cursor + 1U <= plen) {
        uint8_t reportId = payload[cursor];

        if (reportId == S2_BASE_TIMESTAMP_REF || reportId == S2_TIMESTAMP_REBASE) {
            cursor += 5U;
        }
        else if (reportId == S2_GAME_ROTATION_VECTOR && cursor + 12U <= plen) {
            int16_t i = (int16_t)(payload[cursor + 4U] | ((uint16_t)payload[cursor + 5U] << 8));
            int16_t j = (int16_t)(payload[cursor + 6U] | ((uint16_t)payload[cursor + 7U] << 8));
            int16_t k = (int16_t)(payload[cursor + 8U] | ((uint16_t)payload[cursor + 9U] << 8));
            int16_t r = (int16_t)(payload[cursor + 10U] | ((uint16_t)payload[cursor + 11U] << 8));

            dbg_s2_grv++;
            s2_real = r / 16384.0f;
            s2_i    = i / 16384.0f;
            s2_j    = j / 16384.0f;
            s2_k    = k / 16384.0f;
            cursor += 12U;
        }
        else {
            break;
        }
    }
}

static void BNO08x2_Init(void)
{
    int attempt = 0;
    while (!spihal2_reset()) {
        char dbg[32];
        int len = snprintf(dbg, sizeof(dbg), "S2 reset fail #%d\r\n", ++attempt);
        HAL_UART_Transmit(&huart1, (uint8_t *)dbg, (uint16_t)len, 100);
        HAL_Delay(500);
    }

    static uint8_t rx[320];
    s2_resetComplete = false;
    uint32_t start = HAL_GetTick();
    while (!s2_resetComplete && (HAL_GetTick() - start < 500U)) {
        uint16_t n = spihal2_read(rx, sizeof(rx));
        if (n > 0U) {
            s2_handlePacket(rx, n);
        }
    }

    uint8_t req[17] = {0};
    req[0] = S2_SET_FEATURE_CMD;
    req[1] = S2_GAME_ROTATION_VECTOR;
    uint32_t interval = 10000U;
    req[5] = (uint8_t)(interval & 0xFFU);
    req[6] = (uint8_t)((interval >> 8) & 0xFFU);
    req[7] = (uint8_t)((interval >> 16) & 0xFFU);
    req[8] = (uint8_t)((interval >> 24) & 0xFFU);
    dbg_s2_send_ret = s2_send(S2_CHAN_SENSORHUB_CONTROL, req, sizeof(req));
}

static void BNO08x2_Service(void)
{
    static uint8_t rx[320];
    uint16_t n = spihal2_read(rx, sizeof(rx));
    if (n > 0U) {
        s2_handlePacket(rx, n);
    }
}

static void BNO08x_Init(void)
{
    sh2_Hal_t *pHal = sh2_hal_init();

    int attempt = 0;
    while (sh2_open(pHal, NULL, NULL) != SH2_OK) {
        char dbg[32];
        int len = snprintf(dbg, sizeof(dbg), "sh2_open fail #%d\r\n", ++attempt);
        HAL_UART_Transmit(&huart1, (uint8_t *)dbg, (uint16_t)len, 100);
        HAL_Delay(500);
    }

    sh2_setSensorCallback(sensorHandler, NULL);

    sh2_SensorConfig_t config = {0};
    config.reportInterval_us = 10000U;
    dbg_s1_cfg_ret = sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
}

static void sh2_service_wrapper(void)
{
    sh2_service();
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
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  BNO08x_Init();
  BNO08x2_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    if (HAL_GPIO_ReadPin(BNO_INT1_GPIO_Port, BNO_INT1_Pin) == GPIO_PIN_RESET) dbg_int1_low++;
    if (HAL_GPIO_ReadPin(BNO_INT2_GPIO_Port, BNO_INT2_Pin) == GPIO_PIN_RESET) dbg_int2_low++;

    sh2_service_wrapper();
    BNO08x2_Service();

    static uint32_t lastReportAt = 0;
    if (HAL_GetTick() - lastReportAt >= 100U) {
        lastReportAt = HAL_GetTick();
        char tx_buf[128];
        int len = snprintf(tx_buf, sizeof(tx_buf),
            "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n",
            s1_real, s1_i, s1_j, s1_k,
            s2_real, s2_i, s2_j, s2_k);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 100);
    }

    static uint32_t lastDbgAt = 0;
    if (HAL_GetTick() - lastDbgAt >= 1000U) {
        lastDbgAt = HAL_GetTick();
        char dbg_buf[160];
        int dlen = snprintf(dbg_buf, sizeof(dbg_buf),
            "DBG s1cfg=%d s2send=%d s1ev=%lu s1grv=%lu s2pkt=%lu s2grv=%lu int1lo=%lu int2lo=%lu\r\n",
            dbg_s1_cfg_ret, dbg_s2_send_ret,
            (unsigned long)dbg_s1_events, (unsigned long)dbg_s1_grv,
            (unsigned long)dbg_s2_pkts, (unsigned long)dbg_s2_grv,
            (unsigned long)dbg_int1_low, (unsigned long)dbg_int2_low);
        HAL_UART_Transmit(&huart1, (uint8_t *)dbg_buf, (uint16_t)dlen, 100);
    }
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, BNO_CS1_Pin | BNO_CS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, BNO_RST1_Pin | BNO_WAKE1_Pin | BNO_RST2_Pin | BNO_WAKE2_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BNO_CS1_Pin | BNO_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BNO_INT1_Pin | BNO_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BNO_RST1_Pin | BNO_WAKE1_Pin | BNO_RST2_Pin | BNO_WAKE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
