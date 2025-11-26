/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>
#include <ssd1306.h>
#include <ssd1306_fonts.h>
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
// ADC Measurements
uint16_t ADC1Array[2];  // Array to store ADC readings for voltage and current
uint16_t ADC2Array[5];
// Sliding Window (Moving Average) for voltage and current
#define WINDOW_SIZE 100  // Number of readings to average
uint16_t adc_voltage_buffer	[WINDOW_SIZE] = { 0 }; // Buffer for voltage readings
uint16_t adc_current_buffer[WINDOW_SIZE] = { 0 }; // Buffer for current readings
uint8_t adc_index = 0;  // Current index for buffer
uint16_t smoothed_ADC1Array[2]; // Array to store the smoothed voltage and current values
char tx_buffer[43]; // Buffer to store received data
char data_buffer[43]; // Buffer to do string operations


unsigned int is_up_pressed = 0; // 'u'
unsigned int is_right_pressed = 0; // 'r'
unsigned int is_down_pressed = 0; // 'd'
unsigned int is_left_pressed = 0;	// 'l'
unsigned int is_e_pressed = 0; // 'e'
unsigned int is_f_pressed = 0;	// 'f'
unsigned int is_joystick_pressed = 0; // 'j'
unsigned int x_axis_adc0 = 0; // 'x'
unsigned int y_axis_adc1 = 0;	// 'y'


// OLED Display
char buffer[20]; // String buffer for formatted output on the OLED screen

// RGB LED (WS2812)
uint16_t WS2812_RGB_Buff[74] = { 0 }; // Buffer for RGB LED data
int blue_brightness = 0;  // Initial brightness value
int direction = 1; // 1 = increasing, -1 = decreasing

bool has_captured_ultrasonic_rising_edge = false;
uint32_t rising_edge_instant = 0;
uint32_t falling_edge_instant = 0;

#define ECHO_Pin GPIO_PIN_6
#define ECHO_GPIO_Port GPIOC

#define TRIG_Pin GPIO_PIN_7
#define TRIG_GPIO_Port GPIOC

// Delay counter
uint32_t ms_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Motor control function
void motor(int32_t left, int32_t right) {
	// Clamp the values to be within -65535 and +65535, 2^16
	left = fminf(fmaxf(left, -65535), 65535);
	right = fminf(fmaxf(right, -65535), 65535);

	// Handle the left wheel
	if (left >= 0) {
		// Forward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left); // Forward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // Backward
	} else {
		// Backward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // Forward
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -left); // Backward
	}

	// Handle the right wheel
	if (right >= 0) {
		// Forward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, right); // Forward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // Backward
	} else {
		// Backward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0); // Forward
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, -right); // Backward
	}
}
// WS2812 set color
void WS2812_Set(uint8_t R, uint8_t G, uint8_t B) {
	for (uint8_t i = 0; i < 8; i++) {
		//Fill the array
		WS2812_RGB_Buff[i] = (G << i) & (0x80) ? 60 : 29;
		WS2812_RGB_Buff[i + 8] = (R << i) & (0x80) ? 60 : 29;
		WS2812_RGB_Buff[i + 16] = (B << i) & (0x80) ? 60 : 29;
	}
}
// WS2812 initialization
void WS2812_Init() {
	WS2812_Set(255, 255, 255);

	//Use DMA to move the contents from memory to the timer comparison register
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t*) WS2812_RGB_Buff,
			sizeof(WS2812_RGB_Buff) / sizeof(uint16_t));
}
// Breathing blue light function (no loop inside, to be called continuously)
void breathing_blue_update() {
	// Set the LED color (R = 0, G = 0, B = blue_brightness)
	WS2812_Set(0, 0, blue_brightness);

	// Update the brightness value
	blue_brightness += direction * 5; // Adjust the step size (5) for smoothness and speed

	// Reverse the direction at the limits (0 and 50)
	if (blue_brightness >= 50) {
		direction = -1;  // Start decreasing brightness
		blue_brightness = 50;  // Clamp to 50 to avoid overflow
	} else if (blue_brightness <= 0) {
		direction = 1;   // Start increasing brightness
		blue_brightness = 0;  // Clamp to 0 to avoid underflow
	}
}

// Timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Update the breathing light
	if (htim->Instance == TIM13) {
		breathing_blue_update();
	}
}

uint32_t ultrasonic_counter = 0;

float distance = 0.0f;
bool is_working = false;

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM8) {
//		if (!has_captured_ultrasonic_rising_edge) {
//			rising_edge_instant = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			has_captured_ultrasonic_rising_edge = true;
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
//            // HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
//		} else if (has_captured_ultrasonic_rising_edge) {
//			falling_edge_instant = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			has_captured_ultrasonic_rising_edge = false;
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//			__HAL_TIM_SET_COUNTER(htim, 0);  // reset counter
//			HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
//			// __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
//			distance = (((falling_edge_instant > rising_edge_instant) ? 0 : 0xFFFF) + falling_edge_instant - rising_edge_instant) * 0.017;  // cm
//
//			ultrasonic_counter = 0;
//			HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
//			is_working = false;
//		}
//	}
//}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			//__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC1);
		}
	}
}

// Callback function of SysTick
void HAL_SYSTICK_Callback(void) {
//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // 1ms toggle pin
	ms_count++;
	ultrasonic_counter++;

//	// Tesing only
//	if (ms_count < 2000) {
//		// motor(20000, 20000); // Motor ON for 2000 ms
//	} else if (ms_count < 4000) {
//		motor(0, 0); // Motor OFF for 2000 ms
//	} else {
//		ms_count = 0; // Reset counter after 4000 ms
//	}

//	unsigned int is_up_pressed = 0; // 'u'
//	unsigned int is_right_pressed = 0; // 'r'
//	unsigned int is_down_pressed = 0; // 'd'
//	unsigned int is_left_pressed = 0;	// 'l'
	if (is_up_pressed == 1) {
		motor(20000, 20000);
	} else if (is_right_pressed == 1) {
		motor(0, 20000);
	} else if (is_down_pressed == 1) {
		motor(-20000, -20000);
	} else if (is_left_pressed == 1) {
		motor(20000, 0);
	} else {
		motor(0, 0);
	}
}

// ADC Callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1)  // Check which ADC triggered the interrupt
			{
		// Update the ADC buffers with the new readings from DMA
		adc_voltage_buffer[adc_index] = ADC1Array[0]; // Store voltage ADC reading
		adc_current_buffer[adc_index] = ADC1Array[1]; // Store current ADC reading

		// Move the index forward, wrapping around when reaching the buffer size
		adc_index = (adc_index + 1) % WINDOW_SIZE;

		// Calculate the moving average for voltage
		uint32_t voltage_sum = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			voltage_sum += adc_voltage_buffer[i];
		}
		smoothed_ADC1Array[0] = voltage_sum / WINDOW_SIZE; // Store smoothed voltage

		// Calculate the moving average for current
		uint32_t current_sum = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			current_sum += adc_current_buffer[i];
		}
		smoothed_ADC1Array[1] = current_sum / WINDOW_SIZE; // Store smoothed current
	}
}

void substr(char *dest, const char *src, unsigned int start, unsigned int count) {
	/*
		 //  char s[] = "121111110231023" ;
		 //  char t[1];
		 //  substr(t, s, 1 , 1 );
	*/
    strncpy(dest, src + start, count);
    dest[count] = 0;
}

// substring(dest, "hello", 1, 3); // prints "el"
void substring(char *dest, const char *src, unsigned int start, unsigned int end_exclusive) {
	/*
		 //  char s[] = "121111110231023" ;
		 //  char t[1];
		 //  substr(t, s, 1 , 1 );
	*/
	substr(dest, src, start, end_exclusive - start);
}

// char a[] = "a:1 b:2";
void parse_usart_incoming_stream(const char* stream, unsigned int length) {
    char fields[] = { 'u', 'r', 'd', 'l', 'e', 'f', 'j', 'x', 'y' };

    int i = 0;
    int j = 0;
    while (i < length) {
    	for (int ei = 0; ei < sizeof(fields); ei++) {
    		if (stream[i] == fields[ei]) {
    			j = i + 2; //
                for (;j <length; j++) {
                    if (stream[j] == ' ' || stream[j] == '\n') {
                        break;
                    }
                }
                memset(data_buffer,'\0', sizeof(data_buffer));
                substring(data_buffer, stream, i + 2, j);
                printf("%c: ->%s<-, ", stream[i], data_buffer);
                // convert to int and set
                switch(stream[i])
                {
					case 'u':
						if (j - (i + 2 ) != 1) break;
					   sscanf(data_buffer, "%d",&is_up_pressed);
					   break;
					case 'r':
						if (j - (i + 2 ) != 1) break;
					   sscanf(data_buffer, "%d",&is_right_pressed);
					   break;
					case 'd':
						if (j - (i + 2 ) != 1) break;
					   sscanf(data_buffer, "%d",&is_down_pressed);
					   break;
					case 'l':
						if (j - (i + 2 ) != 1) break;
					   sscanf(data_buffer, "%d",&is_left_pressed);
					   break;
					case 'e':
						if (j - (i + 2 ) != 1) break;
					   sscanf(data_buffer, "%d",&is_e_pressed);
					   break;
					case 'f':
						if (j - (i + 2 ) != 1) break;
					   sscanf(data_buffer, "%d",&is_f_pressed);
					   break;
					case 'j':
						if (j - (i + 2 ) != 1) break;
						sscanf(data_buffer, "%d",&is_joystick_pressed);
					   break;
					case 'x':
						if (j - (i + 2 ) != 4) break;
						sscanf(data_buffer, "%04d",&x_axis_adc0);
						break;
					case 'y':
						if (j - (i + 2 ) != 4) break;
						sscanf(data_buffer, "%04d",&y_axis_adc1);
						break;
                }
    			break;
    		}
    	}
        i++;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if(huart->Instance == USART2) {
		parse_usart_incoming_stream(tx_buffer, size);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)&tx_buffer, sizeof(tx_buffer));
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//}

char tracker_marking(uint16_t adc_value) {
	// mid-point of 12-bit ADC
	return adc_value < 2048 ? '-' : '*';
}

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim8, 0);
	while (__HAL_TIM_GET_COUNTER (&htim8) < time);
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC1);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM13_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	// Start right PWM channels
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	// Start left PWM channels
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	// Example on count encoder pulses using interrupts
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);

	// ADC Values for voltage and current
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1Array, 2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) ADC2Array, 5);

	HAL_TIM_Base_Start_IT(&htim13); // LED PB5

	HAL_ADC_Start_IT(&hadc1); // ADC interrupt handler
	HAL_ADC_Start_IT(&hadc2); // ADC interrupt handler

	HAL_TIM_IC_Init(&htim8);
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);



	ssd1306_Init();
	ssd1306_Fill(White);
	ssd1306_UpdateScreen();
	ssd1306_Fill(Black);

	WS2812_Init();

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("RobotCar", Font_11x18, White);
	ssd1306_UpdateScreen();

	HAL_Delay(1000);

	memset(tx_buffer,'\0', sizeof(tx_buffer));
	memset(data_buffer,'\0', sizeof(data_buffer));


    // HAL_UART_Receive_IT(&huart2, (uint8_t *)&tx_buffer, sizeof(tx_buffer));

	HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)&tx_buffer, sizeof(tx_buffer));

    ultrasonic_counter = 0;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		ssd1306_Fill(Black);

//		sprintf(buffer, "%.2fV%s%.3fA",
//				(smoothed_ADC1Array[0] * 0.00459228515 + 0.22), // Voltage calculation
//				(smoothed_ADC1Array[0] * 0.00459228515 + 0.22) < 7.5 ?
//						"TooLow" : "", // Check if voltage is lower than 7.5V
//				((smoothed_ADC1Array[1] * 0.8 - 2500) / 185) < 0 ?
//						0 : ((smoothed_ADC1Array[1] * 0.8 - 2500) / 185)); // Current calculation
//
//		ssd1306_SetCursor(0, 0);  // Set cursor to the top of the display
//		ssd1306_WriteString(buffer, Font_11x18, White);
//

		/*
		snprintf(buffer, sizeof(buffer), "L: %"PRIu32"", __HAL_TIM_GET_COUNTER(&htim2)); // 4,294,967,295
		ssd1306_SetCursor(0, 0); // Set cursor below the voltage/current display
		ssd1306_WriteString(buffer, Font_11x18, White);

		snprintf(buffer, sizeof(buffer), "R: %"PRIu32"", __HAL_TIM_GET_COUNTER(&htim5)); // 4,294,967,295
		ssd1306_SetCursor(0, 30); // Set cursor below the GPIO states
		ssd1306_WriteString(buffer, Font_11x18, White);
		*/

//		if (ultrasonic_counter > 1000 && !is_working) {
//			is_working = true;
//
//			HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
//			__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC1);
//			__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//		}

		HCSR04_Read();

		snprintf(buffer, sizeof(buffer), "%c%c%c%c%c-%dcm",
				tracker_marking(ADC2Array[0]),
				tracker_marking(ADC2Array[1]),
				tracker_marking(ADC2Array[2]),
				tracker_marking(ADC2Array[3]),
				tracker_marking(ADC2Array[4]),
				Distance
		);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(buffer, Font_11x18, White);

		// [STM32 UART Receive via IDLE Line – Interrupt & DMA Tutorial](https://controllerstech.com/stm32-uart-5-receive-data-using-idle-line/)
		snprintf(buffer, sizeof(buffer), "%04d, %04d", x_axis_adc0, y_axis_adc1); // 4,294,967,295
		ssd1306_SetCursor(0, 30); // Set cursor below the GPIO states
		ssd1306_WriteString(buffer, Font_11x18, White);

		ssd1306_UpdateScreen();
		// Write your code below

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
