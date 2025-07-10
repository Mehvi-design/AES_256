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
#include "string.h"
/* USER CODE END Includes */

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
CRYP_HandleTypeDef hcryp;
__ALIGN_BEGIN static const uint8_t pKeyAES[32] __ALIGN_END = {
                            0x60,0x3D,0xEB,0x10,0x15,0xCA,0x71,0xBE,0x2B,0x73,
                            0xAE,0xF0,0x85,0x7D,0x77,0x81,0x01,0x02,0x05,0x60,
                            0x03,0x00,0x60,0x78,0x0A,0xAB,0xAC,0xA6,0xA8,0xA9,
                            0xB6,0xD7};
__ALIGN_BEGIN static const uint8_t pInitVectAES[16] __ALIGN_END = {
                            0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
                            0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_AES_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t plain_text[32] = "hello1234567890ABCDEFGHIJKLMNOP"; // pad to 16 bytes
uint8_t encrypted_text[32];

uint8_t user_encrypted_text[32];
char rx_buffer[32];
// Function to convert a byte to two ASCII hex characters


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
  MX_AES_Init();
  /* USER CODE BEGIN 2 */
  // Encrypt
  if (HAL_CRYP_AESCBC_Encrypt(&hcryp,plain_text,32 , encrypted_text, HAL_MAX_DELAY) != HAL_OK) {
  	                             Error_Handler();
  	                         }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  memset(rx_buffer, 0, sizeof(rx_buffer));
	  memset(user_encrypted_text, 0, sizeof(user_encrypted_text));

	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Enter Password: ", strlen("Enter Password: "), HAL_MAX_DELAY);
	      uint8_t idx = 0;
	      uint8_t ch;

	                    while (1)
	                    {
	                        // Receive one character
	                        HAL_UART_Receive(&hlpuart1, &ch, 1, HAL_MAX_DELAY);

	                        // Echo the character back
	                        HAL_UART_Transmit(&hlpuart1, &ch, 1, HAL_MAX_DELAY);

	                        // Store into buffer
	                        rx_buffer[idx++] = ch;

	                        // Break on newline or carriage return
	                        if (ch == '\r' || ch == '\n' || idx >= sizeof(rx_buffer) - 1)
	                        {
//	                            rx_buffer[idx] = '\0'; // Null-terminate the string
	                            if(idx<sizeof(rx_buffer)){
	                            	memset(&rx_buffer[idx],0x00,sizeof(rx_buffer)-idx);
	                            }
	                            break;
	                        }
	                    }


	                    // send a newline after the echo
	                    HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	                    if (HAL_CRYP_AESCBC_Encrypt(&hcryp, (uint8_t*)rx_buffer, 32, user_encrypted_text, HAL_MAX_DELAY) != HAL_OK) {
	                             Error_Handler();
	                         }
	                    if(memcmp(user_encrypted_text,encrypted_text,32)!=0){
	                 	   HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Invalid Password", strlen("Invalid Password"), HAL_MAX_DELAY);
	                    }else{
	                    	HAL_UART_Transmit(&hlpuart1, (uint8_t *)"WELCOME!!", strlen("WELCOME!!"), HAL_MAX_DELAY);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
}

/**
  * @brief AES Initialization Function
  * @param None
  * @retval None
  */
static void MX_AES_Init(void)
{

  /* USER CODE BEGIN AES_Init 0 */

  /* USER CODE END AES_Init 0 */

  /* USER CODE BEGIN AES_Init 1 */

  /* USER CODE END AES_Init 1 */
  hcryp.Instance = AES;
  hcryp.Init.DataType = CRYP_DATATYPE_8B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_256B;
  hcryp.Init.OperatingMode = CRYP_ALGOMODE_ENCRYPT;
  hcryp.Init.ChainingMode = CRYP_CHAINMODE_AES_CBC;
  hcryp.Init.KeyWriteFlag = CRYP_KEY_WRITE_ENABLE;
  hcryp.Init.pKey = (uint8_t *)pKeyAES;
  hcryp.Init.pInitVect = (uint8_t *)pInitVectAES;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN AES_Init 2 */

  /* USER CODE END AES_Init 2 */

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

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
