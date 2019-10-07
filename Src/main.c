/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned char txcompleted = 0;
unsigned int rx,tx,pm25,pm25_i,temp,temp_i,wet,wet_i,led,pm25_prestate,buzzer,pm25_state;
uint8_t pm2524_1[48],pm2524_2[48];
/*65,58,64,69,83,96,103,97,
										104,93,107,86,80,76,95,102,127,144,152,150,
										145,143,145,132,123,134,106,83,65,73,86,95,
										99,115,117,128,124,112,97,78,83,108,104,109,
										105,109,104,101*/
uint8_t pm2524[48];
uint8_t temp24[48];
uint8_t wet24[48];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Timer中斷 -----------------------------------------------------------------*/
unsigned int clk_seg7,clk_adc,clk_debounce,clk_buzzer,clk_pm25,clk_fpga;
unsigned int clk_ms,clk_1m,clk_5m,clk_20m,clk_50m,clk_05,clk_1,clk_30min;
/********************************************************
 *	公式->中斷頻率=APB1頻率(32MHz/Prescaler+1/Period+1)	*
 *	htim6.Init.Prescaler = 3199;												*
 *	htim6.Init.Period = 9;															*
 *	每1ms持行一次																				*
 ********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(clk_pm25>0) clk_pm25--;
	if(clk_ms>0)  clk_ms--;
	if(clk_ms==0) {
		clk_ms = 100;
//		if(clk_seg7>0) clk_seg7--;	//七段用中斷
		if(clk_adc>0)  clk_adc--;	//ADC用中斷
		if(clk_buzzer>0) clk_buzzer--;
		
		if(clk_fpga) clk_fpga--;
//		if(clk_debounce>0) clk_debounce--;	//防彈跳用中斷
//		if(clk_1m)     clk_1m--;
//		if(clk_5m>0)   clk_5m--;
//		if(clk_05>0)   clk_05--;
//		if(clk_1>0)    clk_1--; 
		if(clk_30min>0)    clk_30min--; 
	}
} 
//void USART1_IRQHandler (void) 
//{
//	if ( USART1->ISR & USART_ISR_RXNE ) {	// RXNE
//		rxtail = (rxtail + 1) & 0x1f;
//		rxbuf[rxtail] = USART1->RDR;
//	}
//	if ( USART1->ISR & USART_ISR_TC )	{		// TC
//		USART1->ICR = USART_ICR_TCCF;
//		txcompleted = 1;
//	}
//}

/* ADC -----------------------------------------------------------------------*/
unsigned int adc[4]={0,0,0,0},ADC_Value[200];				//ADC的值0~4095,ADC取多次做平均的陣列
unsigned int ana0,ana1,ana2,ana3,adc_i,adc_averge_i;//ADC的值0~3.3V
unsigned int adc_vol(unsigned int channel)					//讀取ADC
{
	//channel=ADC_CHSELR_CHSEL0,ADC_CHSELR_CHSEL1,ADC_CHSELR_CHSEL2,ADC_CHSELR_CHSEL3,
	/**********************************************************************************
	 * (1) Select HSI16 by writing 00 in CKMODE (reset value) 												*
	 * (2) Select the auto off mode 																									*
	 * (3) Select CHSEL17 for VRefInt																									*
	 * (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us *
	 * (5) Wake-up the VREFINT (only for Temp sensor and VRefInt) 										*
	 **********************************************************************************/
	//ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; 															/* (1) */
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;		 															/* (2) */
	ADC1->CHSELR = channel;																						/* (3) */
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; 	/* (4) */
	ADC->CCR |= ADC_CCR_VREFEN; 																			/* (5) */
	/* Performs the AD conversion */
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
	while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* wait end of conversion */
	{
	/* For robust implementation, add here time-out management */
	}
	
	return ADC1->DR;	//回傳ADC的值
}

void adconverter(void)	//ADC做平均
{
//	if(clk_adc==0)
//	{
//		clk_adc=1;	//以1ms中斷
		ADC_Value[adc_i++] = adc_vol(ADC_CHSELR_CHSEL0);	//將通道0的值丟入ADC_Value%4=0的格子
		adc_i &= 0x7f;
		if(adc_averge_i <= 0x7f)
		{
			adc[0] += ADC_Value[adc_averge_i++];	//所有ADC_Value%4=0的總和丟入adc[0]
		}
		else
		{
			adc[0] >>= 7;  //將adc[0]取平均
			ana0 = ((float)adc[0]*(2.75/4095)) *1000; //將adc[0]轉換成0~3.3V*1000
			adc[0] = 0;
			adc_averge_i = 0;
//		}
	}
}


char sbuf[10];
char rxbuf[32], rxhead, rxtail;
char txbuf[32], txhead, txtail;
unsigned int a;
void Tx_Task()
{
//	if((GPIOA->IDR & 0x0010) != 0){
		while ( txhead != txtail ) {
			if ( USART2->ISR & USART_ISR_TC )	{		// TC
				USART2->ICR = USART_ICR_TCCF;
				txhead = (txhead + 1) & 0x1f;
				USART2->TDR = txbuf[txhead];
			}
		}
//	}
}
void Tx_UART1()
{
//	if((GPIOA->IDR & 0x0010) != 0){
		while ( txhead != txtail ) {
			if ( USART1->ISR & USART_ISR_TC )	{		// TC
				USART1->ICR = USART_ICR_TCCF;
				txhead = (txhead + 1) & 0x1f;
				USART1->TDR = txbuf[txhead];
			}
		}
//	}
}
void Echo_Char(char ch)
{
	txtail = txhead = 0;
	txtail = (txtail + 1) & 0x1f;
	txbuf[txtail] = ch;
}

void Echo_Str(char *s)
{
	txtail = txhead = 0;
	while ( *s != '\0' ) {
		txtail = (txtail + 1) & 0x1f;
		txbuf[txtail] = *s;
		s++;
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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);						//開啟Timer中斷
	
	rxtail = rxhead = 0;
	txtail = txhead = 0;
	pm25_i = temp_i = wet_i = 0;
	clk_30min = 10000;
	buzzer=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	pm25_state = 0;
//	Echo_Char(0x41);
//	Tx_UART1();

//	for(int i=0;i<256;i++){
//		Echo_Char(i);
//		Tx_Task();
//	}	
//	for(int i=0;i<256;i++){
//		Echo_Char(i);
//		Tx_Task();
//	}		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(clk_pm25==0){
			if(pm25_state==0){
				clk_pm25 = 28;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
				pm25_state = 1;
			}
			else if(pm25_state==1){
				clk_pm25 = 4;
				adconverter();
				pm25 = (0.17*ana0 - 0.1*1000);
				if(pm25>255) pm25=255;
				if(pm25>250 & pm25_prestate<=250){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
					Echo_Str("A");
					Tx_UART1();
				}
				if(pm25<250 & pm25_prestate>=250){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
					Echo_Str("B");
					Tx_UART1();
					
				}
				if(pm25>250){
					if(clk_buzzer==0 & buzzer<=5){
						clk_buzzer = 100;
						HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
						buzzer++;
					}
				}
				else{
					if(buzzer>5){
						buzzer=0;
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
					}
				}
				pm25_prestate = pm25;
				if(clk_fpga==0){
					clk_fpga=1000;
					GPIOC->ODR = pm25;
				}
				pm25_state = 2;
			}
			else if(pm25_state==2){
				clk_pm25 = 968;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
				pm25_state = 0;
			}
		}
			
				
		wet  = (GPIOB->IDR >> 0)&0xff;
		wet  = ((wet>>4)&0xf)*10 + (wet&0xf);
		temp = (GPIOB->IDR >> 8)&0xff;
		temp = ((temp>>4)&0xf)*10 + (temp&0xf);
		if(temp>68 & wet>28){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
			Echo_Str("A");
			Tx_UART1();
			if(clk_buzzer==0 & buzzer<=5){
				clk_buzzer = 100;
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
				buzzer++;
			}
			else{
				if(buzzer>5){
					buzzer=0;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
				}
			}
		}
		
		if(clk_30min == 0){
			clk_30min = 1800000;
//			pm2524_2[pm25_i] = pm2524_1[pm25_i];
//			pm2524_1[pm25_i] = pm2524[pm25_i];
			pm2524[pm25_i] = pm25;
//			if((pm2524_2[pm25_i]>pm2524_2[pm25_i-1])&&(pm2524_1[pm25_i]>pm2524_1[pm25_i-1])&&(pm2524[pm25_i]>pm2524[pm25_i-1])) led=1;;
			pm25_i++;
			pm25_i %= 48;
			temp24[temp_i++] = temp;
			temp_i %= 48;
			wet24[wet_i++] = wet;
			wet_i %= 48;
		}
		if(USART2->ISR & USART_ISR_RXNE) {	// RXNE
			rxtail = (rxtail + 1) & 0x1f;
			rxbuf[rxtail] = USART2->RDR;
			rx = USART2->RDR;
			if(rx == 0x72){
				for(int i=0;i<48;i++){
					Echo_Char(pm2524[pm25_i++]);
					pm25_i %= 48;
					Tx_Task();
					
					Echo_Char(wet24[wet_i++]);
					wet_i %= 48;
					Tx_Task();
					Echo_Char(temp24[temp_i++]);
					temp_i %= 48;
					Tx_Task();
				}
				Echo_Char(clk_30min/60000);
				Tx_Task();
			}
			else if(rx == 0x6E){
				Echo_Char(pm25);
				Tx_Task();
				Echo_Char(wet);
				Tx_Task();
				Echo_Char(temp);
				Tx_Task();
			}
			else if(rx == 0x6F){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
				Echo_Str("A");
				Tx_UART1();
			}
			else if(rx == 0x66){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
				Echo_Str("B");
				Tx_UART1();
			}
		}
//		if(led==1){
//			Echo_Char(0x41);
//			Tx_UART1();
//		}
//		else {
//			Echo_Char(0x42);
//			Tx_UART1();
//		}
//		
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
