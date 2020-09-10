/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <bme280.h>
#include <rfm95.h>
#include <lorawan.h>
#include <secconfig.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct pins {
  GPIO_TypeDef *port;
  uint32_t     pin;
} pin_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

// BME280 driver requires SPI read/write interface functions to use only one uint8 as device id.
// STM32 pins in LL libs are defined by port and pin so pin alone cannot be used as device id.
// To be able to reuse the same SPI functions for RFM and BME280, I use a dev -> port/pin mapping table
pin_t pins[] = {
  { BME_CS_GPIO_Port, BME_CS_Pin },
  { RFM_CS_GPIO_Port, RFM_CS_Pin },
  { RFM_D0_GPIO_Port, RFM_D0_Pin },
  { RFM_D5_GPIO_Port, RFM_D5_Pin }
};

// must correspond to array index above
typedef enum {
  BME280_CS_PIN_ID,
  RFM95_NSS_PIN_ID,
  RFM95_DIO0_PIN_ID,
  RFM95_DIO5_PIN_ID
} pin_id_t;

struct bme280_dev bme280_dev;
struct bme280_data bme280_data;
uint32_t bme280_delay;

rfm95_t rfm95_dev;
uint8_t rfm95_ver;
lorawan_t lorawan;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void putstr( const char *str ) {
  LL_LPUART_Enable(LPUART1);
  while (*str) {
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1));
    LL_LPUART_TransmitData8(LPUART1, (uint8_t)*str++);
  }
  while (!LL_LPUART_IsActiveFlag_TC(LPUART1));
  LL_LPUART_Disable(LPUART1);
}


void putul( unsigned long u ) {
  char num[11];
  char *d = &num[sizeof(num)-1];
  *d = '\0';
  if( u ) {
    while(u) {
      *(--d) = '0' + u % 10;
      u /= 10;
    }
  }
  else {
    *(--d) = '0';
  }
  putstr(d);
}


void puthex( uint8_t val ) {
  static char hex[] = "0123456789abcdef";
  char msg[3];
  msg[0] = hex[val >> 4];
  msg[1] = hex[val & 0xf];
  msg[3] = '\0';
  putstr(msg);
}


int8_t duplexSpi(uint8_t dev, uint8_t addr, uint8_t *data, uint16_t len) {
  LL_SPI_Enable(SPI1);
  LL_GPIO_ResetOutputPin(pins[dev].port, pins[dev].pin);

  LL_SPI_TransmitData8(SPI1, addr);

  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));   // wait for rx to finish
  LL_SPI_ReceiveData8(SPI1);                 // avoid overflow

  while (len--) {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));  // wait for tx to finish
    LL_SPI_TransmitData8(SPI1, *data);       // send data or generate clock for slave

    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)); // wait for rx to finish
    *data = LL_SPI_ReceiveData8(SPI1);       // read receive result
    data++;
  }

  while (LL_SPI_IsActiveFlag_BSY(SPI1));
  LL_GPIO_SetOutputPin(pins[dev].port, pins[dev].pin);
  LL_SPI_Disable(SPI1);

  return 0;
}


uint8_t readPin(uint8_t dev) {
  return LL_GPIO_IsInputPinSet(pins[dev].port, pins[dev].pin);
}


uint8_t *serialize(uint8_t *data, uint32_t value, size_t size) {
  while( size-- ) {
    *(data++) = value & 0xff;
    value >>= 8;
  }
  return data;
}


uint16_t getVdda() {
  LL_ADC_Enable(ADC1);

  while( !LL_ADC_IsActiveFlag_ADRDY(ADC1) );
  LL_ADC_REG_StartConversion(ADC1);
  while( !LL_ADC_IsActiveFlag_EOC(ADC1) );

  uint16_t mV = LL_ADC_REG_ReadConversionData12(ADC1);
  mV = VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR) / mV;

  while( !LL_ADC_IsActiveFlag_EOS(ADC1) );
  LL_ADC_Disable(ADC1);

  return mV;
}


void bme_setup() {
  bme280_dev.dev_id = BME280_CS_PIN_ID;
  bme280_dev.intf = BME280_SPI_INTF;
  bme280_dev.read = duplexSpi;
  bme280_dev.write = duplexSpi;
  bme280_dev.delay_ms = LL_mDelay;

  int bme_init = 0;

  while( !bme_init ) {
    if (bme280_init(&bme280_dev) == BME280_OK) {
      bme280_dev.settings.osr_h = BME280_OVERSAMPLING_1X;
      bme280_dev.settings.osr_p = BME280_OVERSAMPLING_16X;
      bme280_dev.settings.osr_t = BME280_OVERSAMPLING_2X;
      bme280_dev.settings.filter = BME280_FILTER_COEFF_16;
      if( bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &bme280_dev) == BME280_OK ) {
        bme280_delay = bme280_cal_meas_delay(&bme280_dev.settings);
        putstr(" BME delayMs:"); putul(bme280_delay);
        if( bme280_delay ) {
          bme_init = 1;
        }
      }
      else {
        putstr(" BME setup error!\n");
      }
    }
    else {
      putstr(" BME init error!\n");
    }

    if( ! bme_init ) {
      LL_mDelay(1000);
    }
  }
}


int bme_read() {
  bme280_data.humidity = 0;
  if( bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280_dev) == BME280_OK ) {
    bme280_dev.delay_ms(bme280_delay);
    bme280_get_sensor_data(BME280_ALL, &bme280_data, &bme280_dev);
  }

  return bme280_data.humidity >         0 && bme280_data.humidity  <= 100000
      && bme280_data.pressure >     81000 && bme280_data.pressure  <  110000
      && bme280_data.temperature > -50000 && bme280_data.temperature <= 9500;
}


void bme_print() {
  putstr(" T:");      putul(bme280_data.temperature);
  putstr(" c°C, H:"); putul(bme280_data.humidity);
  putstr(" m%, P:");  putul(bme280_data.pressure);
  putstr(" Pa");
}


void rfm_setup( uint32_t seed ) {
  rfm95_dev.nss_pin_id = RFM95_NSS_PIN_ID;
  rfm95_dev.dio0_pin_id = RFM95_DIO0_PIN_ID;
  rfm95_dev.dio5_pin_id = RFM95_DIO5_PIN_ID;
  rfm95_dev.spi_write = duplexSpi;
  rfm95_dev.spi_read = duplexSpi;
  rfm95_dev.delay = LL_mDelay;
  rfm95_dev.pin_read = readPin;

  while( rfm95_ver != 0x12 ) {
    rfm95_ver = rfm95_init(&rfm95_dev, seed);
  }

  lorawan_init(&lorawan, &rfm95_dev);
  lorawan_set_keys(&lorawan, NwkSkey, AppSkey, DevAddr);

  // Print test package
  if( 0 ) {
      uint8_t buf[8];
      for( size_t i=0; i<sizeof(buf); i++ ) {
        buf[i] = i;
      }

    lorawan_send_data(&lorawan, buf, sizeof(buf), 0);
  }
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
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
  LL_GPIO_SetOutputPin(pins[BME280_CS_PIN_ID].port, pins[BME280_CS_PIN_ID].pin);
  LL_GPIO_SetOutputPin(pins[RFM95_NSS_PIN_ID].port, pins[RFM95_NSS_PIN_ID].pin);

  putstr("\nStart");

  uint16_t frame_counter = (uint16_t)HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);

  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
    putstr(" from standby");
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  }
  else {
    putstr(" from reset");
  }

  rfm_setup(frame_counter);
  bme_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    putstr(" standbys:"); putul(frame_counter);

    uint16_t mV = getVdda();
    putstr(" mV:"); putul(mV);

    if( bme_read() ) {
      bme_print();

      uint8_t Data[sizeof(bme280_data)+sizeof(mV)];
      uint8_t *data = Data;
      data = serialize(data, mV, sizeof(mV));
      data = serialize(data, (uint32_t)bme280_data.temperature, sizeof(bme280_data.temperature));
      data = serialize(data, bme280_data.humidity, sizeof(bme280_data.humidity));
      serialize(data, bme280_data.pressure, sizeof(bme280_data.pressure));

      putstr(" Send ");
      putul(lorawan_send_data(&lorawan, Data, sizeof(Data), frame_counter)/1000);
      putstr(" kHz");
    }

    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, ++frame_counter);

    HAL_PWREx_EnableUltraLowPower();
    HAL_PWREx_EnableFastWakeUp();
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTANDBYMode();

    putstr(" Standby ERROR\n");

    LL_mDelay(60000);

    HAL_DeInit();
    NVIC_SystemReset();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
  /** Common config
  */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_LOW);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);

   /* Enable ADC internal voltage regulator */
   LL_ADC_EnableInternalRegulator(ADC1);
   /* Delay for ADC internal voltage regulator stabilization. */
   /* Compute number of CPU cycles to wait for, from delay in us. */
   /* Note: Variable divided by 2 to compensate partially */
   /* CPU processing cycles (depends on compilation optimization). */
   /* Note: If system core clock frequency is below 200kHz, wait time */
   /* is only a few CPU processing cycles. */
   uint32_t wait_loop_index;
   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
   while(wait_loop_index != 0)
     {
   wait_loop_index--;
     }
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**LPUART1 GPIO Configuration
  PA1   ------> LPUART1_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  LPUART_InitStruct.BaudRate = 115200;
  LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
  LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
  LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
  LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX;
  LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
  LL_LPUART_EnableHalfDuplex(LPUART1);
  LL_LPUART_DisableRTSHWFlowCtrl(LPUART1);
  LL_LPUART_DisableIT_CTS(LPUART1);
  LL_LPUART_EnableCTSHWFlowCtrl(LPUART1);
  LL_LPUART_DisableIT_ERROR(LPUART1);
  LL_LPUART_DisableCTSHWFlowCtrl(LPUART1);
  LL_LPUART_IsActiveFlag_CTS(LPUART1);
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 290;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BME_CS_GPIO_Port, BME_CS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(RFM_CS_GPIO_Port, RFM_CS_Pin);

  /**/
  GPIO_InitStruct.Pin = RFM_D0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RFM_D0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BME_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BME_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RFM_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RFM_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RFM_D5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RFM_D5_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
