/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
//#include "usb_otg.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bh1750_config.h"
#include <stdio.h>
#include <string.h>

#include <arm_math.h> /// jesli podkresla to zainstaluj cmsis dsp

#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Tp 0.01 /// czestotliwosc wyswietlania
//#define k 1  // do symulacji
//#define T 5  /// do symulacji

#define kp 40		///trzeba dobrac eksperymentalnie nastawy
#define ki 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int duty=0;

float SWV_VAR; /// do podgladu na wykresie
//arm_pid_instance_f32 S; /// tworzymy obiekt pid
arm_pid_instance_f32 S = {.Kp = kp, .Ki = ki};
float light;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
float calc_pwm(float val)
{
    const float k = 0.13f;
    const float x0 = 70.0f;
    return 10000.0f / (1.0f + exp(-k * (val - x0)));
}

void PID_init(void)///przywiazanie wartosci do obiektu
{
	S.Kp = kp;
	S.Ki = ki*Tp;
	arm_pid_init_f32(&S, 1);
}
float32_t PID_control(float we) ///funkcja z wartoscia zadana i liczenie uchybu
{
	static float y = 0; ///wywoluje sie tylko raz
	float error = we - y;
	//y = model(arm_pid_f32(&S, error)); // zamiast modelu to ustawiamy wartosc wypelnienia pwm __HAL_TIM_Set_Compare(timer,jaki kanal,arm_pid_f32(&S, error))
	y=__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,arm_pid_f32(&S, error));
	return y;
}
//
//void SWV(float value) /// dla symulacyjnego pokazania
//{
//	for(int i = 0; i < 10*(1/Tp); i++)
//	{
//		SWV_VAR = PID_control(value);
//		HAL_Delay(1000*Tp);
//	}
//}
//int __io_putchar(int ch)
//{
//  if (ch == '\n') {
//    __io_putchar('\r');
//  }
//
//  HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
//
//  return 1;
//}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t single_message_recived [30]={0};
	int var=0;
	char single_message_response [30]={0};

	HAL_StatusTypeDef uart3_recived_status ;
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
  MX_USART3_UART_Init();
  //MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  BH1750_Init(&hbh1750_1);


//  PID_init();
//  PID_control(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//

//Komunikacja z bh1750
	  light = BH1750_ReadIlluminance_lux(&hbh1750_1);

	  char msg[32] = { 0, };
	  int msg_len = sprintf(msg, "Illuminance:  %d [lx]\r\n", (int)light);
	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, msg_len, 100);
	  HAL_Delay(1000);
	  //to jest nasza wartość aktualna


	  //miejsce na przyjęcie wiadomości z uart
//	  uart3_recived_status = HAL_UART_Receive (& huart3 , ( uint8_t *)single_message_recived ,strlen("999"), 1000);
//	  single_message_recived [4]= ' \0 ';
//
//	  if ( uart3_recived_status == HAL_OK )
//	  {
//		  sscanf((char*) single_message_recived, "%i" , &var);
//	  }
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);

//	  sprintf(single_message_response , "uart reciveed %s\r\n " ,
//	  single_message_recived);
//
//	  HAL_UART_Transmit (& huart3 , ( uint8_t *) single_message_response ,
//	  strlen ( single_message_response) , 10000) ;

	  //miesjce na regulator


	  //generacja PWM
//	  	  r = 50 * (1.0f + sin(counter / 100.0f));
//	  //	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, counter1);
//	  	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, calc_pwm(r)/8);
//
//	  	  r1=calc_pwm(r)/8;
//	  	  HAL_Delay(1);
//
//	  	 counter++;







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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
