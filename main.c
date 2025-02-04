/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
#include "stdio.h"
#include "string.h"
#include "knowledge_audio.h"
#include "NanoEdgeAI_Audio.h"
#include "knowledge_smoke.h"
#include "NanoEdgeAI_Smoke.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 256
#define AUDIO_AXIS 1
#define SMOKE_AXIS 1
#define CONFIRMATIONS_NB   (uint32_t)(3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Audio sensor variables
float audio;
float audio_buffer[BUFFER_SIZE * AUDIO_AXIS] = {0};
float output_class_buffer_audio[CLASS_NUMBER_AUDIO];
const char *id2class_audio[CLASS_NUMBER_AUDIO + 1] = {
    "unknown", "DOG_BARKING", "GLASS_BREAKING", "ROOM_NOISE", "DOOR_BREAKING"
};

// Smoke sensor variables
float smoke;
float smoke_buffer[BUFFER_SIZE * SMOKE_AXIS] = {0};
float output_class_buffer_smoke[CLASS_NUMBER_SMOKE];
const char *id2class_smoke[CLASS_NUMBER_SMOKE + 1] = {
    "unknown", "SMOKE_DETECTED", "NO_SMOKE"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Inference_Audio();
void Inference_Smoke();
void fill_audio_buffer();
void fill_smoke_buffer();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        audio = HAL_ADC_GetValue(&hadc1);
        smoke = HAL_ADC_GetValue(&hadc1);
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
    enum neai_state error_code;

    /* Initialize the audio model */
    error_code = neai_classification_init_audio(knowledge_audio);
    if (error_code != NEAI_OK) {
        printf("Audio Model Initialization Failed! Error Code: %d\r\n", error_code);
    } else {
        printf("Audio Model Initialization Successful!\r\n");
    }

    /* Initialize the smoke model */
    error_code = neai_classification_init_smoke(knowledge_smoke);
    if (error_code != NEAI_OK) {
        printf("Smoke Model Initialization Failed! Error Code: %d\r\n", error_code);
    } else {
        printf("Smoke Model Initialization Successful!\r\n");
    }
    /* USER CODE END Init */

     /* Configure the system clock */
     SystemClock_Config();

     /* USER CODE BEGIN SysInit */

     /* USER CODE END SysInit */

     /* Initialize all configured peripherals */

    /* MCU Configuration--------------------------------------------------------*/

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();

    /* USER CODE BEGIN 2 */
    HAL_ADC_Start_IT(&hadc1);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        Inference_Audio();
        HAL_Delay(100); // Short delay between readings
        Inference_Smoke();
        HAL_Delay(100);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

       /** Enable MSI Auto calibration
       */
       HAL_RCCEx_EnableMSIPLLMode();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LD3_Pin */
    GPIO_InitStruct.Pin = LD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
  }


/* USER CODE BEGIN 4 */
void fill_audio_buffer() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        audio_buffer[i * AUDIO_AXIS] = audio;
        HAL_Delay(3);
    }
}

void fill_smoke_buffer() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        smoke_buffer[i * SMOKE_AXIS] = smoke;
        HAL_Delay(3);
    }
}

void Inference_Audio() {
    uint16_t id_class_audio;
    fill_audio_buffer();
    neai_classification_audio(audio_buffer, output_class_buffer_audio, &id_class_audio);
    printf("Audio Detected Class: %s\r\n", id2class_audio[id_class_audio]);

    if (strcmp(id2class_audio[id_class_audio], "DOOR_BREAKING") == 0 ||
        strcmp(id2class_audio[id_class_audio], "GLASS_BREAKING") == 0) {
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET);
    }
}

void Inference_Smoke() {
    uint16_t id_class_smoke;
    fill_smoke_buffer();
    neai_classification_smoke(smoke_buffer, output_class_buffer_smoke, &id_class_smoke);
    printf("Smoke Detected Class: %s\r\n", id2class_smoke[id_class_smoke]);

    if (strcmp(id2class_smoke[id_class_smoke], "SMOKE_DETECTED") == 0) {
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    }
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */


void Error_Handler(void) {
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

