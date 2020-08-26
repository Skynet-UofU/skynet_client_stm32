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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

//initialize methods
void SystemClock_Config(void);

//UART methods
void transmit_bt_char(char c);
void transmit_bt(char* chars);
void transmit_lora_char(char c);
void transmit_lora(char* chars);
void initializeUART2();
void initializeUART1();

//Global variables used to keep track of complete messages
char* msg;
int bytes_received = 0;



/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
	
  /* Configure the system clock */
  SystemClock_Config();
	
	GPIOC->CRH |=  (1<<20)| (1<<21); // set pc13 to output mode, max speed 50 MHz
	//GPIOC->CRH |= (1<<22); // set output to open-drain
	
	GPIOC->ODR |= (1<<13);

//	initializeUART2();
//	initializeUART1();
	
//	//set the GPIO settings for the LEDS
//	GPIOC->MODER |= 0x55000; //set PC8 and PC9 to general output, PC6 & 7 to be alternate function mode	
	
//	transmit_bt("Starting\r\n");
	__HAL_RCC_GPIOC_CLK_ENABLE();
//	transmit_lora("This is a test to see just how many characters we can purely send using LoRa without having to deal with the bluetooth uart limitations. will it end up being the bt, or the lora that causes the bottleneck?");
	while (1)
  {				
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			HAL_GPIO_TogglePin(GPIOC, 13);

      HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}


//The LoRa module is using uart2, so when LoRa data comes in,
//relay it to the bluetooth app
void USART2_IRQHandler(void)
{
	char characters[256];
	uint32_t character_iter = 0;
	while((USART2->SR >> 5) & 1)
	{
		characters[character_iter] = USART2->DR;
		character_iter++;
	}
	characters[character_iter] = 0;
	transmit_bt(characters);
}

//The bluetooth module is using uart1, so when data comes in,
//relay it to the LoRa module
void USART1_IRQHandler(void)
{
	char characters[256];
	uint32_t character_iter = 0;
	while((USART1->SR >> 5) & 1)
	{
		characters[character_iter] = USART1->DR;
		character_iter++;
		if((USART1->SR >> 3) & 1)
			transmit_bt("overrun occured");
	}
	characters[character_iter] = 0;
	transmit_bt(characters);
}

void transmit_lora_char(char c)
{
	while(!(USART2->SR & (1 << 7))) //check for the transmission to be clear for the next byte
	{
	}
	USART2->DR = c; //set the character to be transmitted
}

void transmit_lora(char* chars)
{
	//loop through all the characters checking for null and an invalid pointer
	uint32_t counter = 0;
	while(chars != 0 && chars[counter] != 0)
	{
		transmit_lora_char(chars[counter]);
		counter++;
	}
}


void transmit_bt_char(char c)
{
	while(!(USART1->SR & (1 << 7))) //check for the transmission to be clear for the next byte
	{
	}
	USART1->DR = c; //set the character to be transmitted
}

void transmit_bt(char* chars)
{
	//loop through all the characters checking for null and an invalid pointer
	uint32_t counter = 0;
	while(chars != 0 && chars[counter] != 0)
	{
		transmit_bt_char(chars[counter]);
		counter++;
	}
}

char convert_to_ascii(uint8_t val)
{
	char ret = 0;
	ret = val >= 0xa ? (val - 0xa) + 'a' : val + '0';
	return ret;
}

void convert_to_ascii_string(uint8_t val, char* ret)
{
	ret[0] = convert_to_ascii(val >> 4);
	ret[1] = convert_to_ascii(val & 0xf);
	ret[2] = 0;
}

void toggle_LED(uint8_t pin)
{
	GPIOC->ODR |= (1 << pin);
	HAL_Delay(250);
	GPIOC->ODR &= ~(1 << pin);
	HAL_Delay(250);
	GPIOC->ODR |= (1 << pin);
	HAL_Delay(250);
	GPIOC->ODR &= ~(1 << pin);
}

// The LoRa module is connected to uart2
// uart2 on the blue pill uses PA2 and PA3
void initializeUART2(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable system clock in RCC peripheral
	__HAL_RCC_AFIO_CLK_ENABLE();
	GPIOA->CRL &= ~(0xF00); // set PA2 and PA3 to alternate function mode
	GPIOA->CRH |= 0xA00;
	
//	GPIOA->AFR[1] &= ~(0xFF000000); // make sure all other bits are zeroes
//	GPIOA->AFR[1] |= (1 << 24) | (1 << 28); //set PA14 & PA15 to use AF1
	
	// set PA9 and PA10 to use AF1
	// should be set to USART1 by default
		
	USART2->BRR = HAL_RCC_GetHCLKFreq()/115200; //set the baud rate to 9600
	
	USART2->CR1 |= (1 << 3) | (1 << 2) | (1 << 5); //enable the transmit and receive
	
	USART2->CR2 |= (1<<12);

  NVIC_EnableIRQ(USART2_IRQn); //enable the USART2 interrupt
  NVIC_SetPriority(USART2_IRQn,1); //set the USART2 priority
	
	USART2->CR1 |= 1; //enable the USART2 peripheral, this must be done after all other settings are configured	
}

// The Bluetooth module is connected to uart1
void initializeUART1(void)
{
	RCC->APB1ENR |= RCC_APB2ENR_USART1EN; // enable system clock in RCC peripheral
//	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIOA->CRH &= ~(0xF); // set PA9 and PA10 to alternate function mode
	GPIOA->CRH |= 0xA;
		
	
	USART1->BRR = HAL_RCC_GetHCLKFreq()/115200; // set Baud rate to be 115200 bits/second
	
	USART1->CR1 |= ((1<<3) | (1<<2)); // enable tx/rx hardware
	USART1->CR1 |= (1<<5); // enable rx reg not empty interrupt
	USART1->CR1 |= 1;			 // enable USART3
	
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 2);

//  // set pins pc10 and pc11 into alternate function mode for UART3_TX/RX
//	GPIOC->AFR[1] |= (1<<8); 
//	GPIOC->AFR[1] |= (1<<12);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
