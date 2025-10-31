/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "crypto.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Structure pour stocker les informations de temps et de date du RTC
typedef struct
	{
	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;
	uint8_t Month;
	uint8_t Date;
	uint8_t Year;
	}Struct_RTC;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Nom du fichier dans lequel les données chiffrées seront stockées
#define FILENAME "ENCRYP_FILE.txt"

// Taille du texte en clair (avant chiffrement) en octets
// Ici : 16 octets = taille typique pour un bloc AES-128
#define PLAINTEXT_LENGTH 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADC_Task */
osThreadId_t ADC_TaskHandle;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for SDIO_Task */
osThreadId_t SDIO_TaskHandle;
const osThreadAttr_t SDIO_Task_attributes = {
  .name = "SDIO_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for RTC_Task */
osThreadId_t RTC_TaskHandle;
const osThreadAttr_t RTC_Task_attributes = {
  .name = "RTC_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for Encrypt_Task */
osThreadId_t Encrypt_TaskHandle;
const osThreadAttr_t Encrypt_Task_attributes = {
  .name = "Encrypt_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for Decrypt_Task */
osThreadId_t Decrypt_TaskHandle;
const osThreadAttr_t Decrypt_Task_attributes = {
  .name = "Decrypt_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for RTC_Queue */
osMessageQueueId_t RTC_QueueHandle;
const osMessageQueueAttr_t RTC_Queue_attributes = {
  .name = "RTC_Queue"
};
/* Definitions for ADC_Queue */
osMessageQueueId_t ADC_QueueHandle;
const osMessageQueueAttr_t ADC_Queue_attributes = {
  .name = "ADC_Queue"
};
/* Definitions for Encrypt_DATAQueue */
osMessageQueueId_t Encrypt_DATAQueueHandle;
const osMessageQueueAttr_t Encrypt_DATAQueue_attributes = {
  .name = "Encrypt_DATAQueue"
};
/* Definitions for BinarySemSD_CARD */
osSemaphoreId_t BinarySemSD_CARDHandle;
const osSemaphoreAttr_t BinarySemSD_CARD_attributes = {
  .name = "BinarySemSD_CARD"
};
/* USER CODE BEGIN PV */
FRESULT state;
FRESULT res;
uint16_t ADC_VAL = 0;
uint16_t ADC_VAL_GET = 0;
uint8_t WelcomeMsg[] = "STM32 FATFS works great!\n\r";
FATFS fs;
FIL file;

RTC_TimeTypeDef ActualTime = {0};
RTC_DateTypeDef ActualDate = {0};

Struct_RTC RTC_data;
Struct_RTC RTC_data_received;

uint8_t Plaintext[16] = {0};
AESCBCctx_stt AESctx;
int32_t output_enc_length = 0;
int32_t output_dec_length = 0;

uint8_t OutputEncMessage[40];
uint8_t OutputDecMessage[40];
uint32_t OutputMessageLength = 0;

uint8_t data[40];

char EncodedHexbuff[70];
char Rec_EncodedHexbuff[70];
uint8_t showpress[30];
uint32_t counter = 0;
char SD_W_Buff[70];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_RTC_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);
void Get_ADC_Val(void *argument);
void Write_TO_SDIO(void *argument);
void Get_Time_Date(void *argument);
void Encrypt_DATA(void *argument);
void Decrypt_DATA(void *argument);

/* USER CODE BEGIN PFP */

// Clé de chiffrement symétrique (AES)
uint8_t Key[] =
  {
    0x70, 0x3a, 0xec, 0x12, 0x14, 0xca, 0x54, 0xad,
    0x3b, 0x65, 0x11, 0xfe, 0xff, 0x7a, 0x75, 0x83,
    0x2f, 0x36, 0x2e, 0x03, 0x3d, 0x62, 0x09, 0x7d,
  };

// Vecteur d'initialisation (IV) utilisé pour AES en mode CBC
uint8_t IV[] =
  {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
  };
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Convertit un tableau de bytes en chaîne hexadécimale.
void EncodeHex(uint8_t *input, uint32_t length, char *output)
{
    for (uint32_t i = 0; i < length; i++) {
        sprintf(&output[i * 2], "%02X", input[i]);
    }
    output[length * 2] = '\0'; // null-terminate
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		counter++;
		if(osSemaphoreRelease(BinarySemSD_CARDHandle)!=osOK)
		{
		   Error_Handler();
		}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySemSD_CARD */
  BinarySemSD_CARDHandle = osSemaphoreNew(1, 1, &BinarySemSD_CARD_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of RTC_Queue */
  RTC_QueueHandle = osMessageQueueNew (6, sizeof(Struct_RTC), &RTC_Queue_attributes);

  /* creation of ADC_Queue */
  ADC_QueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &ADC_Queue_attributes);

  /* creation of Encrypt_DATAQueue */
  Encrypt_DATAQueueHandle = osMessageQueueNew (1, sizeof(EncodedHexbuff), &Encrypt_DATAQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ADC_Task */
  ADC_TaskHandle = osThreadNew(Get_ADC_Val, NULL, &ADC_Task_attributes);

  /* creation of SDIO_Task */
  SDIO_TaskHandle = osThreadNew(Write_TO_SDIO, NULL, &SDIO_Task_attributes);

  /* creation of RTC_Task */
  RTC_TaskHandle = osThreadNew(Get_Time_Date, NULL, &RTC_Task_attributes);

  /* creation of Encrypt_Task */
  Encrypt_TaskHandle = osThreadNew(Encrypt_DATA, NULL, &Encrypt_Task_attributes);

  /* creation of Decrypt_Task */
  Decrypt_TaskHandle = osThreadNew(Decrypt_DATA, NULL, &Decrypt_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x15;
  sTime.Minutes = 0x36;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x17;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Get_ADC_Val */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Get_ADC_Val */
void Get_ADC_Val(void *argument)
{
  /* USER CODE BEGIN Get_ADC_Val */
  /* Infinite loop */
  for(;;)
  {
	 if(HAL_ADC_Start(&hadc1)!=HAL_OK)
	 {
	  	Error_Handler();
	 }
	 if(HAL_ADC_PollForConversion(&hadc1, 100)!=HAL_OK)
	 {
	  	Error_Handler();
	 }
	 ADC_VAL = HAL_ADC_GetValue(&hadc1);
	 if(HAL_ADC_Stop(&hadc1)!=HAL_OK)
	 {
	  	Error_Handler();
	 }
	 if(osMessageQueuePut(ADC_QueueHandle, &ADC_VAL, NULL, osWaitForever)!=osOK)
	 {
		 Error_Handler();
	 }
    osDelay(1);
  }
  /* USER CODE END Get_ADC_Val */
}

/* USER CODE BEGIN Header_Write_TO_SDIO */
/**
* @brief Function implementing the SDIO_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Write_TO_SDIO */

//Tâche FreeRTOS d’écriture sur carte SD via SDIO
void Write_TO_SDIO(void *argument)
{
  /* USER CODE BEGIN Write_TO_SDIO */
	uint32_t byteswritten;
	char showtime[30];
	char showdate[30];
	char adc_tab[30];
  /* Infinite loop */
  for(;;)
  {
	  // Récupération du buffer hexadécimal depuis la queue (bloc jusqu'à réception)
	  if(osMessageQueueGet(Encrypt_DATAQueueHandle, &Rec_EncodedHexbuff, NULL, osWaitForever)!=osOK)
	  {
	  	 Error_Handler();
	  }
	  // Format du numéro d’événement (presses bouton)
	  sprintf(showpress,"%d: ",counter);
	  // Ajout CR/LF après les données pour le fichier texte
	  snprintf(SD_W_Buff,sizeof(SD_W_Buff),"%s\r\n",Rec_EncodedHexbuff);
	  // Attendre que le sémaphore SD soit disponible (libéré par ISR sur bouton)
	  if(osSemaphoreAcquire(BinarySemSD_CARDHandle, osWaitForever)!=osOK)
	  {
		 Error_Handler();
	  }
	  // Initialisation interface SD
	  if( HAL_SD_Init(&hsd)!=HAL_OK)
	  {
	  	 Error_Handler();
	  }
	  // Montage du système de fichiers FATFS
	  if(f_mount(&fs, "", 1)!=FR_OK)
	  {
	  	 Error_Handler();
	  }
	  // Ouvrir fichier en mode append (création si inexistant)
	  if(f_open(&file, "SD_DATA.txt", FA_WRITE | FA_OPEN_APPEND)!=FR_OK)
	  {
	  	 Error_Handler();
	  }
	  // Écrire l'index d'événement
	  if(f_write(&file, showpress, sizeof(showpress), &byteswritten)== FR_OK)
	  {
	  	 //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	  	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	  }
	  // Écrire les données chiffrées
	  if(f_write(&file, SD_W_Buff, sizeof(SD_W_Buff), &byteswritten)== FR_OK)
	  {
	  	  	 //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	  	  	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
	  // Forcer l'écriture sur la carte
	  if(f_sync(&file)!= FR_OK)
	  {
	  	 Error_Handler();
	  }
	  // Fermer le fichier
	  if(f_close(&file)!= FR_OK)
	  {
	  	 Error_Handler();
	  }
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    osDelay(1);
  }
  /* USER CODE END Write_TO_SDIO */
}

/* USER CODE BEGIN Header_Get_Time_Date */
/**
* @brief Function implementing the RTC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Get_Time_Date */
void Get_Time_Date(void *argument)
{
  /* USER CODE BEGIN Get_Time_Date */

  /* Infinite loop */
  for(;;)
  {
	  // Lire l'heure depuis le RTC (format BCD)
	  HAL_RTC_GetTime(&hrtc, &ActualTime, RTC_FORMAT_BCD);
	  // Lire la date depuis le RTC
	  HAL_RTC_GetDate(&hrtc, &ActualDate, RTC_FORMAT_BCD);
	  // Stocker les données temporelles dans la structure utilisateur
	  RTC_data.Hours = ActualTime.Hours;
	  RTC_data.Minutes = ActualTime.Minutes;
	  RTC_data.Seconds = ActualTime.Seconds;
	  RTC_data.Date = ActualDate.Date;
	  RTC_data.Month = ActualDate.Month;
	  RTC_data.Year = ActualDate.Year;
	  // Envoyer les données RTC à la file de messages pour traitement dans une autre tâche
	  if(osMessageQueuePut(RTC_QueueHandle, &RTC_data, NULL, osWaitForever)!=osOK)
      {
		 Error_Handler();
	  }
    osDelay(1);
  }
  /* USER CODE END Get_Time_Date */
}

/* USER CODE BEGIN Header_Encrypt_DATA */
/**
* @brief Function implementing the Encrypt_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Encrypt_DATA */
void Encrypt_DATA(void *argument)
{
  /* USER CODE BEGIN Encrypt_DATA */

	// Activer l'horloge du module CRC utilisé par la librairie de chiffrement
	  __CRC_CLK_ENABLE();
	  // Configuration du contexte AES
	  AESctx.mFlags = E_SK_DEFAULT;
	  AESctx.mKeySize = 24;
	  AESctx.mIvSize = sizeof(IV);
	  uint16_t year = 0;
	  uint32_t data_length =0;
  /* Infinite loop */
  for(;;)
  {
	  // Récupérer les données RTC depuis la queue
	  if(osMessageQueueGet(RTC_QueueHandle, &RTC_data_received, NULL, osWaitForever)!=osOK)
	  {
		 Error_Handler();
	  }
	  // Récupérer la valeur ADC depuis la queue
	  if(osMessageQueueGet(ADC_QueueHandle, &ADC_VAL_GET, NULL, osWaitForever)!=osOK)
	  {
		 Error_Handler();
	  }
	  // Convertir l'année BCD (exemple : 0x24 → 2024)
	  year = 2000+RTC_data_received.Year;
	  // Formater les données en chaîne lisible : Date, Heure, valeur ADC
	  snprintf(data, sizeof(data), "%02d-%02d-%04d %02d:%02d:%02d, ADC_VAL=%04d",
			   RTC_data_received.Date,
	           RTC_data_received.Month,
			   year,
	           RTC_data_received.Hours,
	           RTC_data_received.Minutes,
	           RTC_data_received.Seconds,
	           ADC_VAL_GET);
	      data_length = strlen(data);
	      // Initialiser le chiffrement AES CBC
	      if(AES_CBC_Encrypt_Init(&AESctx, Key, IV)!=AES_SUCCESS)
	  	  {
	  		  Error_Handler();
	  	  }
	      // Chiffrer le message
	  	  if(AES_CBC_Encrypt_Append(&AESctx, data, data_length, OutputEncMessage, &output_enc_length)!=AES_SUCCESS)
	  	  {
	  		  Error_Handler();
	  	  }
	  	  // Finaliser l’opération (gestion du padding)
	  	  OutputMessageLength = output_enc_length;
	  	  if(AES_CBC_Encrypt_Finish(&AESctx, OutputEncMessage + output_enc_length, &output_enc_length)!=AES_SUCCESS)
	  	  {
	  		  Error_Handler();
	  	  }
	  	  OutputMessageLength += output_enc_length;
	  	  EncodeHex(OutputEncMessage, OutputMessageLength, EncodedHexbuff);
	  	  // Envoyer le message chiffré à la file Encrypt_DATAQueue
	  	  if(osMessageQueuePut(Encrypt_DATAQueueHandle, &EncodedHexbuff, NULL, osWaitForever)!=osOK)
	  	  {
	  		 Error_Handler();
	  	  }

    osDelay(1);
  }
  /* USER CODE END Encrypt_DATA */
}

/* USER CODE BEGIN Header_Decrypt_DATA */
/**
* @brief Function implementing the Decrypt_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Decrypt_DATA */
void Decrypt_DATA(void *argument)
{
  /* USER CODE BEGIN Decrypt_DATA */
  /* Infinite loop */
  for(;;)
  {
	      //Initialiser le contexte de déchiffrement AES-CBC
	      if(AES_CBC_Decrypt_Init(&AESctx, Key, IV)!=AES_SUCCESS)
	       {
	     	  Error_Handler();
	       }
	      // Déchiffrer les données chiffrées (OutputEncMessage)
	 	  if(AES_CBC_Decrypt_Append(&AESctx, OutputEncMessage, OutputMessageLength, OutputDecMessage, &output_dec_length)!=AES_SUCCESS)
	 	  {
	 	      Error_Handler();
	 	  }
	 	  // Mettre à jour la longueur partielle du message déchiffré
	 	  OutputMessageLength = output_dec_length;
	 	  // Finaliser le déchiffrement pour gérer le padding éventuel
	 	  if(AES_CBC_Decrypt_Finish(&AESctx, OutputDecMessage + output_dec_length, &output_dec_length)!=AES_SUCCESS)
	      {
	     	 Error_Handler();
	      }
	 	  OutputMessageLength += output_dec_length;
    osDelay(1);
  }
  /* USER CODE END Decrypt_DATA */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  while (1)
  {
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
