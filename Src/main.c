/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "../../../Hardware/MPU6050/mpu6050.h"
#include "../../../Hardware/MPU6050/delay.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu_dmp_motion_driver.h" 
#include "stm32f1xx_it.h"
#include <math.h>
#include <stdlib.h>
#include "protocol.h"
#include "PID1.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//int fputc(int ch, FILE *f)
//{
//    uint8_t temp[1]={ch};
//    HAL_UART_Transmit(&huart1, temp, 1, HAL_MAX_DELAY);
//    return ch;
//}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pitch[3],roll[3],yaw[3];//ŷ����
short aacx[3],aacy[3],aacz[3];//���ٶȴ�����ԭʼ����
short gyrox[3],gyroy[3],gyroz[3];//������ԭʼ����
short temp[3];//�¶�
extern float SetAngle1;
extern float SetAngle2;
//float pitch_1,roll_1,yaw_1; 		//ŷ����
//short aacx_1,aacy_1,aacz_1;		//���ٶȴ�����ԭʼ����
//short gyrox_1,gyroy_1,gyroz_1;	//������ԭʼ����
//float pitch_2,roll_2,yaw_2; 		//ŷ����
//short aacx_2,aacy_2,aacz_2;		//���ٶȴ�����ԭʼ����
//short gyrox_2,gyroy_2,gyroz_2;	//������ԭʼ����
//float pitch_3,roll_3,yaw_3; 		//ŷ����
//short aacx_3,aacy_3,aacz_3;		//���ٶȴ�����ԭʼ����
//short gyrox_3,gyroy_3,gyroz_3;	//������ԭʼ����
//short temp1, temp2, temp3;					//�¶�



extern int i, k;
extern uint8_t receivedata;
uint8_t EventFlag;
//int voltage_limit=2500;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define USART_REC_LEN 200
uint8_t		Res;
uint8_t 	USART1_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.��һ���200������������������ӣ�
uint16_t  USART1_RX_STA;       			 				//����״̬���
float voltage_out1=0;
float voltage_out2=0;

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//	HAL_UART_Receive_IT(&huart1, &receivedata, 1);   //ʹ�ܽ����жϴ���1
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);  
  protocol_init();
  Three_MPU6050_Init();
	
	joint1_PID_init(&pid_roll1_structure, &pid_pitch1_structure);
	joint2_PID_init(&pid_roll2_structure, &pid_pitch2_structure);
	joint3_PID_init(&pid_roll3_structure, &pid_pitch3_structure);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    if(EventFlag==0x02){
//			Get_Three_MPU6050_data();
	  	Get_joint3_MPU6050_data();
			joint_motion_3(pid_roll3_structure.SetVal, pid_pitch3_structure.SetVal);
//		}
//      joint_motion_3(10, 10);
//		arm_motion_1();
//		arm_one_motion();//�����˶�����

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

//��ʼ������MPU6050
void Three_MPU6050_Init(void)
{
//��ʼ��MPU6050	1
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	while(MPU_Init());					
	printf("%s\r\n","Mpu_1 Initializing");
	while(mpu_dmp_init())
	{
		delay_ms(200);
		printf("%s\r\n","Mpu_1 Init Wrong!");
	}
	printf("%s\r\n","Mpu_1 Init OK!");
//��ʼ��MPU6050 2
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	while(MPU_Init());					
	printf("%s\r\n","Mpu_2 Initializing");
	while(mpu_dmp_init())
	{
		delay_ms(200);
		printf("%s\r\n","Mpu_2 Init Wrong!");
	}
	printf("%s\r\n","Mpu_2 Init OK!");
//��ʼ��MPU6050 3
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	while(MPU_Init());					
	printf("%s\r\n","Mpu_3 Initializing");
	while(mpu_dmp_init())
	{
		delay_ms(200);
		printf("%s\r\n","Mpu_3 Init Wrong!");
	}
	printf("%s\r\n","Mpu_3 Init OK!");
}

void Get_joint1_MPU6050_data(void)
{
//��ȡMPU6050	1
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	while(mpu_dmp_get_data(&pitch[0],&roll[0],&yaw[0]))
	{
  temp[0]=MPU_Get_Temperature();								//�õ��¶�ֵ
  MPU_Get_Accelerometer(&aacx[0],&aacy[0],&aacz[0]);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox[0],&gyroy[0],&gyroz[0]);	//�õ�����������
	}
}

void Get_joint2_MPU6050_data(void)
{
//��ȡMPU6050	2
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	while(mpu_dmp_get_data(&pitch[1],&roll[1],&yaw[1]))
	{	
  temp[1]=MPU_Get_Temperature();								//�õ��¶�ֵ
  MPU_Get_Accelerometer(&aacx[1],&aacy[1],&aacz[1]);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox[1],&gyroy[1],&gyroz[1]);	//�õ�����������
	}
			int32_t data_1=100*roll[1];
//		int32_t data_3=100*pitch[2];
		set_computer_value(0x02, 0x01, &data_1, 1);		
//		set_computer_value(0x02, 0x02, &data_3, 1);
//	printf("pitch_3=%f  roll_3=%f  \r\n", pitch[2],roll[2]);
}

void Get_joint3_MPU6050_data(void)
{
//��ȡMPU6050	3
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	while(mpu_dmp_get_data(&pitch[2],&roll[2],&yaw[2]))
	{	
  temp[2]=MPU_Get_Temperature();								//�õ��¶�ֵ
  MPU_Get_Accelerometer(&aacx[2],&aacy[2],&aacz[2]);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox[2],&gyroy[2],&gyroz[2]);	//�õ�����������
  }
		int32_t data_1=100*roll[2];
//		int32_t data_3=100*pitch[2];
		set_computer_value(0x02, 0x01, &data_1, 1);		
//		set_computer_value(0x02, 0x02, &data_3, 1);
//	printf("pitch_3=%f  roll_3=%f  \r\n", pitch[2],roll[2]);
}
void Get_Three_MPU6050_data(void)
{

//��ȡMPU6050	1
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	while(mpu_dmp_get_data(&pitch[0],&roll[0],&yaw[0]))
	{
  temp[0]=MPU_Get_Temperature();								//�õ��¶�ֵ
  MPU_Get_Accelerometer(&aacx[0],&aacy[0],&aacz[0]);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox[0],&gyroy[0],&gyroz[0]);	//�õ�����������
	
	}
//	HAL_Delay(100);
	
//��ȡMPU6050	2
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	while(mpu_dmp_get_data(&pitch[1],&roll[1],&yaw[1]))
	{	
  temp[1]=MPU_Get_Temperature();								//�õ��¶�ֵ
  MPU_Get_Accelerometer(&aacx[1],&aacy[1],&aacz[1]);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox[1],&gyroy[1],&gyroz[1]);	//�õ�����������
	
	}
//	HAL_Delay(100);
	
//��ȡMPU6050	3
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	while(mpu_dmp_get_data(&pitch[2],&roll[2],&yaw[2]))
	{	
  temp[2]=MPU_Get_Temperature();								//�õ��¶�ֵ
  MPU_Get_Accelerometer(&aacx[2],&aacy[2],&aacz[2]);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox[2],&gyroy[2],&gyroz[2]);	//�õ�����������
	
  }
//	uint32_t data0=roll[0];
//	uint32_t data1=pitch[0];
//	set_computer_value(0x02, 0x01, &data0, 1);		
//	set_computer_value(0x02, 0x02, &data1, 1);
//	set_computer_value(0x02, 0x01, &data, 1);
//	set_computer_value(0x02, 0x04, roll+1, 1);
//	set_computer_value(0x02, 0x05, pitch+2, 1);
//	set_computer_value(0x02, 0x01, void *data, 1);
//	printf("pitch_1=%f  roll_1=%f  \r\n", pitch[0],roll[0]);
//	printf("pitch_2=%f  roll_2=%f  \r\n", pitch[1],roll[1]);
//	printf("pitch_3=%f  roll_3=%f  \r\n", pitch[2],roll[2]);	
//	HAL_Delay(100);
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    /* �ж����ĸ����ڴ������ж� */
//    if(huart ->Instance == USART1)
//    {
//        		//������ܵ�������
//				if((USART1_RX_STA&0x8000)==0)//����δ���
//				{
//					//��ȡ���յ�������
//					if(Res==0x0D)
//					{
//						USART1_RX_STA|=0x8000;
//						HAL_UART_Receive_IT(&huart1, &Res, 1);
//					}
//					else
//					{
//						USART1_RX_BUF[USART1_RX_STA&0X3FFF]=Res ;
//						USART1_RX_STA++;
//						if(USART1_RX_STA>(USART_REC_LEN-1))
//							USART1_RX_STA=0;
//					}		 
//				}
//				//�ȴ���һ�ν����ж�
//				HAL_UART_Receive_IT(huart,&Res,1);
//    }
//}

void set_p_i_d(pid1_struct *pid,float p, float i, float d)
{
  pid->Kp = p;    // ���ñ���ϵ�� P
  pid->Ki = i;    // ���û���ϵ�� I
  pid->Kd = d;    // ����΢��ϵ�� D
}
void set_pid_target(pid1_struct *pid,float temp_val)
{
  pid->SetVal  = temp_val/100;    // ���õ�ǰ��Ŀ��ֵ
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
