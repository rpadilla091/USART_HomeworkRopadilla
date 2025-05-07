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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart_rx_buffer[RX_BUFFER_SIZE];
char command_buffer[RX_BUFFER_SIZE];
volatile uint8_t command_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void ParseCommand(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    memcpy(command_buffer, uart_rx_buffer, RX_BUFFER_SIZE);

    // Echo back what was received (for debugging)
    HAL_UART_Transmit(&huart3, (uint8_t*)"RX Raw: ", 8, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)command_buffer, RX_BUFFER_SIZE, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

    command_ready = 1;
    HAL_UART_Receive_IT(&huart3, uart_rx_buffer, RX_BUFFER_SIZE);
  }
}


void ParseCommand(char *cmd)
{
    // Trim CR, LF, or garbage past buffer
    for (int i = 0; i < RX_BUFFER_SIZE; i++) {
        if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == '\0') {
            cmd[i] = '\0';
            break;
        }
    }

    // Truncate at the first closing '}'
    char *end = strchr(cmd, '}');
    if (end != NULL) {
        *(end + 1) = '\0';  // Include the closing brace
    }

    HAL_UART_Transmit(&huart3, (uint8_t*)"Parsing...\r\n", 13, HAL_MAX_DELAY);

    // Validate structure
    if (strncmp(cmd, "{LED:", 5) != 0 ||
        cmd[7] != ',' ||
        strncmp(&cmd[8], "STATE:", 6) != 0 ||
        cmd[16] != '}') {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Invalid format\r\n", 17, HAL_MAX_DELAY);
        return;
    }

    // Extract LED ID and state
    char led_id[3] = { cmd[5], cmd[6], '\0' };
    char state[3]  = { cmd[14], cmd[15], '\0' };

    GPIO_PinState pin_state = (strcmp(state, "ON") == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (strcmp(led_id, "01") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, pin_state);   // LD1 Green
        HAL_UART_Transmit(&huart3, (uint8_t*)"Green toggled\r\n", 16, HAL_MAX_DELAY);
    } else if (strcmp(led_id, "02") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, pin_state);   // LD2 Blue
        HAL_UART_Transmit(&huart3, (uint8_t*)"Blue toggled\r\n", 15, HAL_MAX_DELAY);
    } else if (strcmp(led_id, "03") == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, pin_state);  // LD3 Red
        HAL_UART_Transmit(&huart3, (uint8_t*)"Red toggled\r\n", 14, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Unknown LED ID\r\n", 17, HAL_MAX_DELAY);
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

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);   // LD1
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);   // LD2
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);   // LD1
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);   // LD2
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3
	HAL_Delay(1000);

  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  char *startup_msg = "STM32 Ready. Send LED commands...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);
  HAL_UART_Receive_IT(&huart3, uart_rx_buffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (command_ready) {
	    command_ready = 0;

	    HAL_UART_Transmit(&huart3, (uint8_t*)"CMD: ", 5, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&huart3, (uint8_t*)command_buffer, RX_BUFFER_SIZE, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	    ParseCommand(command_buffer);
	  }

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
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14; // Change to your actual LED pins
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
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
