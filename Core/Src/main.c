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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "oled.h"
#include <math.h>
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
uint8_t key=0;//读取按键状态
uint16_t flag_pid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//函数定义
double calculate_temp(double v);
void heat(double t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int ADCValue;

//double temp_voltage;


	int is_heat;
	int set_flag=0;
	
	double temp_gap;//实际温度与设置温度的差值
    double temp_now;
	uint8_t temp_now_integer;
	double temp_now_decimal;
 	
	int temp_set=40;
	int temp_set_last=40;
	
	uint8_t device_mode=0;//0 is off,1 is on,2设置加热温度，3设置温控范围中的最低温度,4设置温控范围中的最高温度
	uint8_t is_clean=0;
	int min_temp=0;
	int max_temp=99;

	uint8_t alert_flag=0;
uint8_t time_counter=0;
char str[]=".";
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
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();    
	OLED_Clear();

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 
	
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1,10);
	  ADCValue=HAL_ADC_GetValue(&hadc1);
	  
	  ADCValue =ADCValue*3300/4095;					//AD值转换为电压值
	  printf("Voltage = %4d mV \r\n",ADCValue);
	 
	  temp_now=calculate_temp(ADCValue/1000.0);
	  
	  printf("Voltage = %.3f 摄氏度 \r\n",temp_now);
	  temp_now_integer=(int) temp_now;
	  temp_now_decimal=(temp_now-temp_now_integer)*10;
	  key = KEY_Scan(0);
				
				
		//进入警报
		if((device_mode==1)&&(temp_now<min_temp||temp_now>max_temp)){
			
				device_mode=5;
				is_clean=1;
	}

	switch(device_mode){
		case 0:
			OLED_ShowCHinese(10,0,0);//状态：
			OLED_ShowCHinese(27,0,1);
			OLED_ShowCHinese(43,0,2);
			OLED_ShowCHinese(50,0,3);//关闭
			OLED_ShowCHinese(67,0,4);
		
			
			OLED_ShowCHinese(10,3,11);//实际温度
			OLED_ShowCHinese(27,3,12);
			OLED_ShowCHinese(44,3,9);
			OLED_ShowCHinese(58,3,10);
			OLED_ShowCHinese(72,3,2);
			OLED_ShowNum(77,3,temp_now,2,16);
			OLED_ShowString(95,3,str,16);
			OLED_ShowNum(100,3,temp_now_decimal,1,16);
			OLED_ShowCHinese(108,3,13);
			switch(key)
			{
				case KEY1_PRES:device_mode=1;is_clean=1;break;
				default:break;
	}
			//OLED_ShowNum(77,0,temp_set,2,16);
			break;
		case 1:
			HAL_Delay(100);
			
			OLED_ShowCHinese(10,0,15);//范围
			OLED_ShowCHinese(27,0,16);
			OLED_ShowNum(44,0,min_temp,2,16);
			OLED_ShowCHinese(60,0,13);
			OLED_ShowCHinese(74,0,17);
			OLED_ShowNum(91,0,max_temp,2,16);
			OLED_ShowCHinese(108,0,13);

			OLED_ShowCHinese(10,3,7);//设置温度
			OLED_ShowCHinese(27,3,8);
			OLED_ShowCHinese(44,3,9);
			OLED_ShowCHinese(58,3,10);
			OLED_ShowCHinese(72,3,2);
			OLED_ShowNum(77,3,temp_set,2,16);
			OLED_ShowCHinese(95,3,13);
		
			OLED_ShowCHinese(10,6,11);//实际温度
			OLED_ShowCHinese(27,6,12);
			OLED_ShowCHinese(44,6,9);
			OLED_ShowCHinese(58,6,10);
			OLED_ShowCHinese(72,6,2);
			OLED_ShowNum(77,6,temp_now,2,16);
			OLED_ShowString(95,6,str,16);
			OLED_ShowNum(100,6,temp_now_decimal,1,16);
			OLED_ShowCHinese(108,6,13);
			switch(key){
				case KEY0_PRES: device_mode=0;is_clean=1; break;
				case KEY1_PRES: device_mode=2;is_clean=1;break;
				case KEY2_PRES: device_mode=3;is_clean=1;break;
				case KEY3_PRES: break;
				default: break;
			}
			
			break;
		case 2://设置加热温度
			OLED_ShowCHinese(10,0,7);//设置
			OLED_ShowCHinese(27,0,8);
			OLED_ShowNum(47,0,temp_set,2,16);
			OLED_ShowCHinese(65,0,13);//加热
			OLED_ShowCHinese(10,3,11);
			OLED_ShowCHinese(27,3,12);
		
			
			OLED_ShowNum(44,3,temp_now,2,16);
			OLED_ShowString(62,3,str,16);
			OLED_ShowNum(67,3,temp_now_decimal,1,16);
		
		
			OLED_ShowCHinese(77,3,13);	
			switch(key){
				case KEY0_PRES: device_mode=0;is_clean=1; break;
				case KEY1_PRES: 
					if(temp_set>min_temp){				
				temp_set--;
					}
					else {
					OLED_ShowCHinese(10,6,32);//不能高于最高值
					OLED_ShowCHinese(27,6,18);
					OLED_ShowCHinese(44,6,21);
					OLED_ShowCHinese(61,6,28);
					OLED_ShowCHinese(78,6,20);
					OLED_ShowCHinese(95,6,21);
					OLED_ShowCHinese(112,6,25);
					
					}
						break;
				case KEY3_PRES: 
					if(temp_set<max_temp){
					temp_set++;
					}
				else{
					OLED_ShowCHinese(10,6,32);//不能低于最低值
					OLED_ShowCHinese(27,6,18);
					OLED_ShowCHinese(44,6,23);
					OLED_ShowCHinese(61,6,28);
					OLED_ShowCHinese(78,6,20);
					OLED_ShowCHinese(95,6,23);
					OLED_ShowCHinese(112,6,25);
				
				}
				
				break;
				case KEY2_PRES: device_mode=1;is_clean=1;break;
				default: break;
			}
			if(temp_set!=temp_set_last){
				temp_set_last=temp_set;
				set_flag=0;
			}
			
			break;
		case 3://设置温控范围最低温
		{
			OLED_ShowCHinese(10,0,9);//温控范围
			OLED_ShowCHinese(27,0,14);
			OLED_ShowCHinese(44,0,15);
			OLED_ShowCHinese(61,0,16);
			OLED_ShowNum(20,3,min_temp,2,16);
			OLED_ShowCHinese(40,3,13);
			OLED_ShowCHinese(60,3,17);
			OLED_ShowNum(80,3,max_temp,2,16);
			OLED_ShowCHinese(100,3,13);
			
			
			
			switch(key){
			case KEY0_PRES: device_mode=0;is_clean=1; break;
			case KEY1_PRES: 
				if(min_temp>0){
					min_temp--;
				}else{
					OLED_ShowCHinese(10,6,20);//最低温为0℃
					OLED_ShowCHinese(27,6,21);
					OLED_ShowCHinese(44,6,9);
					OLED_ShowCHinese(61,6,22);
					OLED_ShowCHinese(78,6,2);
					OLED_ShowNum(85,6,0,1,16);
					OLED_ShowCHinese(95,6,13);
				}	
			break;
				
			case KEY3_PRES: 
				if(min_temp<max_temp-1){
					min_temp++;
				}else{
					OLED_ShowCHinese(10,6,24);//左值应小于右值
					OLED_ShowCHinese(27,6,30);
					OLED_ShowCHinese(44,6,26);
					OLED_ShowCHinese(61,6,27);
					OLED_ShowCHinese(78,6,28);
					OLED_ShowCHinese(95,6,29);
					OLED_ShowCHinese(110,6,30);
				}
							
			break;
				
			case KEY2_PRES: device_mode=4;is_clean=1; break;
			default:break;
			}
		}
		break;
		case 4://设置最高温
		{
			OLED_ShowCHinese(10,0,9);//温控范围
			OLED_ShowCHinese(27,0,14);
			OLED_ShowCHinese(44,0,15);
			OLED_ShowCHinese(61,0,16);
			OLED_ShowNum(20,3,min_temp,2,16);
			OLED_ShowCHinese(40,3,13);
			OLED_ShowCHinese(60,3,17);
			OLED_ShowNum(80,3,max_temp,2,16);
			OLED_ShowCHinese(100,3,13);
			
			
			switch(key){
			case KEY0_PRES: device_mode=0;is_clean=1; break;
			case KEY1_PRES: 
				if(max_temp>min_temp){
					max_temp--;
				}else{
					OLED_ShowCHinese(10,6,29);//右值应大于左值
					OLED_ShowCHinese(27,6,30);
					OLED_ShowCHinese(44,6,26);
					OLED_ShowCHinese(61,6,31);
					OLED_ShowCHinese(78,6,28);
					OLED_ShowCHinese(95,6,24);
					OLED_ShowCHinese(110,6,30);
				}	
			break;
				
			case KEY3_PRES: 
				if(max_temp<=99){
					max_temp++;
				}else{
					OLED_ShowCHinese(10,6,20);//最高温为90℃
					OLED_ShowCHinese(27,6,23);
					OLED_ShowCHinese(44,6,9);
					OLED_ShowCHinese(61,6,22);
					OLED_ShowCHinese(78,6,2);
					OLED_ShowNum(85,6,90,2,16);
					OLED_ShowCHinese(105,6,13);
				}
							
			break;
				
			case KEY2_PRES: device_mode=1;is_clean=1; break;
			default:break;
			}
			}
		break;
			case 5:
				OLED_ShowCHinese(48,1,33);
				OLED_ShowCHinese(65,1,34);
				OLED_ShowNum(20,4,min_temp,2,16);
				OLED_ShowCHinese(40,4,13);
				OLED_ShowCHinese(60,4,17);
				OLED_ShowNum(80,4,max_temp,2,16);
				OLED_ShowCHinese(100,4,13);
			
				if(alert_flag==0){
					switch(key){
					case KEY1_PRES: 
						if(min_temp>0){
							min_temp--;
						}else{
							OLED_ShowCHinese(10,6,20);
							OLED_ShowCHinese(27,6,21);
							OLED_ShowCHinese(44,6,9);
							OLED_ShowCHinese(61,6,22);
							OLED_ShowCHinese(78,6,2);
							OLED_ShowNum(85,6,0,1,16);
							OLED_ShowCHinese(95,6,13);
						}	
				break;
				
			case KEY3_PRES: 
					if(min_temp<max_temp-1){
						min_temp++;
					}else{
						OLED_ShowCHinese(10,6,24);
						OLED_ShowCHinese(27,6,30);
						OLED_ShowCHinese(44,6,26);
						OLED_ShowCHinese(61,6,27);
						OLED_ShowCHinese(78,6,28);
						OLED_ShowCHinese(95,6,29);
						OLED_ShowCHinese(110,6,30);
					}
							
				break;
				case KEY2_PRES: alert_flag=1;break;
					case KEY0_PRES: device_mode=0; is_clean=1 ;break;
				default: break;
					}
				}
				else if(alert_flag==1){
				switch(key){
				case KEY1_PRES: 
					if(max_temp>min_temp){
						max_temp--;
					}else{
						OLED_ShowCHinese(10,6,29);
						OLED_ShowCHinese(27,6,30);
						OLED_ShowCHinese(44,6,26);
						OLED_ShowCHinese(61,6,31);
						OLED_ShowCHinese(78,6,28);
						OLED_ShowCHinese(95,6,24);
						OLED_ShowCHinese(110,6,30);
					}	
					break;
				case KEY3_PRES: 
					if(max_temp<90){
						max_temp++;
					}else{
							OLED_ShowCHinese(10,6,20);
						OLED_ShowCHinese(27,6,23);
						OLED_ShowCHinese(44,6,9);
						OLED_ShowCHinese(61,6,22);
						OLED_ShowCHinese(78,6,2);
						OLED_ShowNum(85,6,90,2,16);
						OLED_ShowCHinese(105,6,13);
					}	
						break;
					case KEY2_PRES: alert_flag=0;break;
					case KEY0_PRES:
				device_mode=0;
				is_clean=1;
				break;
					default:break;
					}
				}

				//HAL_Delay(500);
				is_clean=1;
				
				
			
		default:
				HAL_Delay(10); break;
		}
	
	
		
	//oled屏幕刷新
	if(is_clean)
	{
		is_clean=0;
		OLED_Clear();
	}
		
		
	if(device_mode!=0&&device_mode!=5){
		temp_gap=temp_set-temp_now;
		printf("temp_now= %f,temp_set=%d,temp_gap=%f\r\n,",temp_now,temp_set,temp_gap);
		heat(temp_gap);
	}else{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		printf("关闭状态，不加热");
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,10);
	return ch;
}


void heat(double t)
{
	if(t<-0.3){
			set_flag=1;
		}
			
	if(set_flag==1){
		if(temp_set<=60&&temp_set>=50){
			if(t>2)  
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
				printf("heat,pwm=10000\r\n");
			}
			else if(t<=2&&t>=1)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7500);
				printf("heat,pwm=7500\r\n");
			}
			else if(t<1&&t>0.2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4500);
				printf("heat,pwm=4500\r\n");
			
			}
			else if(t<0.3&&t>-0.2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
				printf("heat,pwm=1000\r\n");
			}
			else{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				printf("heat,pwm=0\r\n");
			}
		}
		else if(temp_set<=50){
			if(t>1.5)  
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
				printf("heat,pwm=10000\r\n");
			}
			else if(t<=1.5&&t>=1)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,6000);
				printf("heat,pwm=6000\r\n");
			}
			else if(t<1&&t>0.2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4500);
				printf("heat,pwm=4500\r\n");
			
			}
			else if(t<0.3&&t>-0.2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
				printf("heat,pwm=1000\r\n");
			}
			else{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				printf("heat,pwm=0\r\n");
			}
		}
		else if(temp_set>60&&temp_set<70){
			if(t>=1.5)  
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
					printf("heat,pwm=10000\r\n");
				}
				else if(t<=1.5&&t>=0.5)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7000);
					printf("heat,pwm=7500\r\n");
				}
				else if(t<0.5&&t>0)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
					printf("heat,pwm=5000\r\n");
				
				}
				else if(t<0.3&&t>-0.3)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
					printf("heat,pwm=2000\r\n");
				}
				else{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
					printf("heat,pwm=0\r\n");
				}
		}
		else if(temp_set>=70&&temp_set<=80){
			if(t>=1.5)  
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,9000);
					printf("heat,pwm=9000\r\n");
				}
				else if(t<=1.5&&t>=0.6)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,6000);
					printf("heat,pwm=6000\r\n");
				}
				else if(t<0.6&&t>0)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
					printf("heat,pwm=5000\r\n");
				
				}
				else if(t<0.3&&t>-0.3)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
					printf("heat,pwm=2000\r\n");
				}
				else{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
					printf("heat,pwm=0\r\n");
				}
		}
		else if(temp_set>80){
			if(t>=1.2)  
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
					printf("heat,pwm=10000\r\n");
				}
				else if(t<=1.2&&t>=0.5)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7000);
					printf("heat,pwm=7500\r\n");
				}
				else if(t<0.5&&t>0)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5000);
					printf("heat,pwm=5000\r\n");
				
				}
				else if(t<0.3&&t>-0.3)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
					printf("heat,pwm=2000\r\n");
				}
				else{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
					printf("heat,pwm=0\r\n");
				}
		}
		
		
	}
	else{
		if(temp_set<=65&&temp_set>55){
			if(t<=0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				printf("heat,pwm=0\r\n");
			}
			else if(t>3.5){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
				printf("heat,pwm=10000\r\n");
			}
			else if(t<3.5&&t>2.4){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5500);
				printf("heat,pwm=5500\r\n");
			}
			else if(t<2.4&&t>0.7){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
				printf("heat,pwm=2000\r\n");
			}
		}
		else if(temp_set>65&&temp_set<=80){
			if(t<=0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				printf("heat,pwm=0\r\n");
			}
			else if(t>3.5){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
				printf("heat,pwm=10000\r\n");
			}
			else if(t<3.5&&t>2.2){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7000);
				printf("heat,pwm=7000\r\n");
			}
			else if(t<2.2&&t>0.7){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5000);
				printf("heat,pwm=5000\r\n");
			}
		}
			else if(temp_set>80){
			if(t<=0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				printf("heat,pwm=0\r\n");
			}
			else if(t>3.5){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
				printf("heat,pwm=10000\r\n");
			}
			else if(t<3.5&&t>2.5){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7000);
				printf("heat,pwm=7000\r\n");
			}
			else if(t<2.5&&t>0.4){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,6000);
				printf("heat,pwm=5000\r\n");
			}
		}
		else if(temp_set<=55&&temp_set>45){
			if(t<=0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				printf("heat,pwm=0\r\n");
			}
			else if(t>6){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
				printf("heat,pwm=10000\r\n");
			}
			else if(t<=6&&t>4){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5000);
				printf("heat,pwm=5000\r\n");
			}
			else if(t<4&&t>2){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4000);
				printf("heat,pwm=3000\r\n");
			}
			else if(t<2&&t>0.5){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2500);
				printf("heat,pwm=2500\r\n");
			}
	}
		else if(temp_set<=45){
			if(t<=0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				printf("heat,pwm=0\r\n");
			}
			else if(t>6){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10000);
				printf("heat,pwm=10000\r\n");
			}
			else if(t<=6&&t>4){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5000);
				printf("heat,pwm=5000\r\n");
			}
			else if(t<4&&t>2){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2500);
				printf("heat,pwm=2500\r\n");
			}
			else if(t<2&&t>0.5){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1300);
				printf("heat,pwm=1300\r\n");
			}
	}
	
	
}
	printf("set_flag=%d\r\n",set_flag);
	}

double calculate_temp(double v){
	double temp_v=v;
	double ratio=temp_v/5;
	double temperature;
	if(ratio<0.9467&&ratio>=0.8462){
		temperature=-777.9*ratio*ratio+1200*ratio-468;
	}else if(ratio<0.8462&&ratio>=0.665){
		temperature=-110.3*ratio+83.35;
	
	}else if(ratio<0.665&&ratio>=0.4462){
		temperature=4.99*ratio*ratio-96.33*ratio+71.95;
	}else if(ratio<0.4462&&ratio>=0.263){
		temperature=101.1*ratio*ratio-180.8*ratio+90.62;
	}else if(ratio<0.263&&ratio>=0.1481){
		temperature=405*ratio*ratio-337.7*ratio+111;
	}else if(ratio<0.1481&&ratio>=0.083){
		temperature=1476*ratio*ratio-647*ratio+133.6;
	}else if(ratio<0.083&&ratio>=0.0483){
		temperature=5226*ratio*ratio-1252*ratio+158.2;
	}
	
	else {
		temperature=10;
	}
	return temperature-3.5;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance==TIM2){
time_counter++;
	
	if(time_counter>9) {
		flag_pid=1;
		time_counter=0;
	}
}

}



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
