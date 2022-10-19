/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TSYS01_ADDR                        0x77  
#define TSYS01_RESET                       0x1E
#define TSYS01_ADC_READ                    0x00
#define TSYS01_ADC_TEMP_CONV               0x48
#define TSYS01_PROM_READ                   0XA0

#define RPM_Calculation                 60000


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

SRAM_HandleTypeDef hsram2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch,  FILE *f){
  
  HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,1);
  
  return ch;
  
}

float temperature_f1 =0.0;

float temperature_f2 =0.0;


uint8_t USART_RX_FLAG = 0;
uint8_t temp_rx =0;

typedef struct{
  uint16_t TIMER_DATA1;
  uint16_t TIMER_DATA2;
  uint16_t TIMER_MINU;
  uint16_t FAN_RPM;
  uint8_t  RPM_check;
}TIMER_struct;



TIMER_struct A1_DATA;
TIMER_struct A2_DATA;
TIMER_struct B1_DATA;
TIMER_struct B2_DATA;



void TIMER_struct_init(TIMER_struct* DATA){
  DATA->TIMER_DATA1=0;
  DATA->TIMER_DATA2=0;
  DATA->TIMER_MINU=0;
  DATA->FAN_RPM=0;
  DATA->RPM_check=1;
  
}

void TIMER_RPM(TIMER_struct* DATA){
  
  if(DATA->RPM_check == 1){
    DATA->TIMER_DATA1=htim6.Instance->CNT;  
    DATA->RPM_check++;
  }
  else if (DATA->RPM_check == 2){
    DATA->RPM_check++;
  }
  else if (DATA->RPM_check == 3){
    DATA->TIMER_DATA2=htim6.Instance->CNT;
    
    if(DATA->TIMER_DATA2 < DATA->TIMER_DATA1) DATA->TIMER_DATA2+=1000;
    
    DATA->TIMER_MINU =  DATA->TIMER_DATA2 - DATA->TIMER_DATA1;
    DATA->FAN_RPM = RPM_Calculation / DATA->TIMER_MINU;
    
    DATA->RPM_check=1;
  }
  
}
 uint8_t i2c_rx[4];
double TEST_I2C(){
uint8_t i2c_tx= 0x00;


    HAL_I2C_Master_Transmit(&hi2c1,0x80,&i2c_tx,1,100);
    HAL_Delay(10);
    HAL_I2C_Master_Receive(&hi2c1,0x80,i2c_rx,4,100);
    HAL_Delay(100);
 return 0.0;
}
double I2C_Temperature(I2C_HandleTypeDef *hi2c){
  
  uint8_t adc_read[5]={0};
  uint8_t adc_k2[2]={0};
  uint8_t adc_k3[2]={0};
  uint8_t adc_k4[2]={0};
  uint8_t adc_k1[2]={0};
  uint8_t adc_k0[2]={0};
  
  uint16_t adc_Coefficient[5]={0};
  uint64_t adc_data=0;
  
  uint16_t adc_read_addr = TSYS01_ADC_READ;
  uint16_t adc_prom_read = TSYS01_PROM_READ;
  uint8_t adc_start_addr = TSYS01_ADC_TEMP_CONV;
  
  
  double temperature_d = 0.0;  
  HAL_I2C_Master_Transmit(hi2c,TSYS01_ADDR<<1,&adc_start_addr,1,100);
  
  HAL_Delay(10);
  HAL_I2C_Mem_Read(hi2c,TSYS01_ADDR<<1,adc_read_addr,1,adc_read,3,100);
  HAL_Delay(10);
  adc_data = 0;
  adc_data = adc_read[0];
  adc_data = (adc_data << 8) | adc_read[1];
  adc_data = (adc_data << 8) | adc_read[2];
  
  
  
  
  adc_data= adc_data/256;
  
  HAL_I2C_Mem_Read(hi2c,TSYS01_ADDR<<1,adc_prom_read+2,1,adc_k4,2,100);
  HAL_Delay(10);
  
  HAL_I2C_Mem_Read(hi2c,TSYS01_ADDR<<1,adc_prom_read+4,1,adc_k3,2,100);
  HAL_Delay(10);
  
  HAL_I2C_Mem_Read(hi2c,TSYS01_ADDR<<1,adc_prom_read+6,1,adc_k2,2,100);
  HAL_Delay(10);   
  
  HAL_I2C_Mem_Read(hi2c,TSYS01_ADDR<<1,adc_prom_read+8,1,adc_k1,2,100);
  HAL_Delay(10);   
  
  HAL_I2C_Mem_Read(hi2c,TSYS01_ADDR<<1,adc_prom_read+10,1,adc_k0,2,100);
  HAL_Delay(10);    
  
  
  
  adc_Coefficient[0] = (uint16_t)((adc_k4[0] << 8) + adc_k4[1]);
  adc_Coefficient[1] = (uint16_t)((adc_k3[0] << 8) + adc_k3[1]);
  adc_Coefficient[2] = (uint16_t)((adc_k2[0] << 8) + adc_k2[1]);
  adc_Coefficient[3] = (uint16_t)((adc_k1[0] << 8) + adc_k1[1]);
  adc_Coefficient[4] = (uint16_t)((adc_k0[0] << 8) + adc_k0[1]);
  
  
  temperature_d = ((-2)*adc_Coefficient[0]/pow(10,21)*pow(adc_data,4))\
    +((4)*adc_Coefficient[1]/pow(10,16)*pow(adc_data,3))\
      +((-2)*adc_Coefficient[2]/pow(10,11)*pow(adc_data,2))\
        +((1)*adc_Coefficient[3]/pow(10,6)*adc_data)\
          +((-1.5)*adc_Coefficient[4]/pow(10,2));
  
  return temperature_d;
  
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  
  
  //FAN_A1_SENSOR
  if(GPIO_Pin == GPIO_PIN_3){
    HAL_GPIO_TogglePin(GPIOC,STATE_LED0_Pin);
    TIMER_RPM(&A1_DATA);  
    
  }
  //FAN_A2_SENSOR
  else if(GPIO_Pin == GPIO_PIN_2){
    TIMER_RPM(&A2_DATA);
    
  }
  //FAN_B1_SENSOR
  else if(GPIO_Pin == GPIO_PIN_5){
    TIMER_RPM(&B1_DATA);
    
  }
  //FAN_B2_SENSOR
  else if(GPIO_Pin == GPIO_PIN_13){
    TIMER_RPM(&B2_DATA);
    
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{  
  
  // USART1 Rx Interrupt
  if (huart->Instance == USART2) {
    
    if(temp_rx == '1'){
      USART_RX_FLAG =1;
    }
    else if(temp_rx == '2'){
      USART_RX_FLAG = 2;
    }
    
    HAL_UART_Receive_IT(&huart2, &temp_rx, 1);
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM6){
    
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
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_FSMC_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &temp_rx, 1);
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint8_t tx_buff[20]=0;
  
  uint8_t i2c_reset = 0x1E;
  
  TIMER_struct_init(&A1_DATA);
  TIMER_struct_init(&A2_DATA);
  TIMER_struct_init(&B1_DATA);
  TIMER_struct_init(&B2_DATA);
  
  // FAN2 Connecter start
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  
  HAL_GPIO_WritePin(GPIOA,FAN2_ONOFF_Pin,GPIO_PIN_SET);
  
  
  //FAN1 Connecter start
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  
  HAL_GPIO_WritePin(GPIOA,FAN1_ONOFF_Pin,GPIO_PIN_SET);   
  
  //rpm data calculation timer
  HAL_TIM_Base_Start_IT(&htim6) ;
  
  HAL_I2C_Master_Transmit(&hi2c3,TSYS01_ADDR<<1,&i2c_reset,1,100);
  HAL_Delay(10);
  
  
  while (1)
  {
    temperature_f1 = I2C_Temperature(&hi2c3);
     temperature_f2 = TEST_I2C();
    
    if(temperature_f1 > 30){
      htim4.Instance->CCR3 = 500-1;
      htim4.Instance->CCR4 = 500-1;
    }
    else {
      htim4.Instance->CCR3 = 900-1;
      htim4.Instance->CCR4 = 900-1;
    }
    
    if(USART_RX_FLAG == 1){
      printf("$temp,1,%.2f,2,%.2f,uart,0\r\n",temperature_f1,temperature_f2);
      HAL_Delay(1000);
      
    }
    else if(USART_RX_FLAG == 2){
      printf("$rpm,A1,%d,A2,%d,B1,%d,B2,%d,uart,1\r\n",A1_DATA.FAN_RPM,A2_DATA.FAN_RPM,B1_DATA.FAN_RPM,B2_DATA.FAN_RPM);
      HAL_Delay(1000);
      
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief I2C1 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C1_Init(void)
{
  
  /* USER CODE BEGIN I2C1_Init 0 */
  
  /* USER CODE END I2C1_Init 0 */
  
  /* USER CODE BEGIN I2C1_Init 1 */
  
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  
  /* USER CODE END I2C1_Init 2 */
  
}

/**
* @brief I2C3 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C3_Init(void)
{
  
  /* USER CODE BEGIN I2C3_Init 0 */
  
  /* USER CODE END I2C3_Init 0 */
  
  /* USER CODE BEGIN I2C3_Init 1 */
  
  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  
  /* USER CODE END I2C3_Init 2 */
  
}

/**
* @brief SPI2 Initialization Function
* @param None
* @retval None
*/
static void MX_SPI2_Init(void)
{
  
  /* USER CODE BEGIN SPI2_Init 0 */
  
  /* USER CODE END SPI2_Init 0 */
  
  /* USER CODE BEGIN SPI2_Init 1 */
  
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  
  /* USER CODE END SPI2_Init 2 */
  
}

/**
* @brief TIM4 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM4_Init(void)
{
  
  /* USER CODE BEGIN TIM4_Init 0 */
  
  /* USER CODE END TIM4_Init 0 */
  
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  /* USER CODE BEGIN TIM4_Init 1 */
  
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
  
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
  htim6.Init.Prescaler = 16000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
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
* @brief TIM13 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM13_Init(void)
{
  
  /* USER CODE BEGIN TIM13_Init 0 */
  
  /* USER CODE END TIM13_Init 0 */
  
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  /* USER CODE BEGIN TIM13_Init 1 */
  
  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 5-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */
  
  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);
  
}

/**
* @brief TIM14 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM14_Init(void)
{
  
  /* USER CODE BEGIN TIM14_Init 0 */
  
  /* USER CODE END TIM14_Init 0 */
  
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  /* USER CODE BEGIN TIM14_Init 1 */
  
  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 2000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
  
  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);
  
}

/**
* @brief UART4 Initialization Function
* @param None
* @retval None
*/
static void MX_UART4_Init(void)
{
  
  /* USER CODE BEGIN UART4_Init 0 */
  
  /* USER CODE END UART4_Init 0 */
  
  /* USER CODE BEGIN UART4_Init 1 */
  
  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  
  /* USER CODE END UART4_Init 2 */
  
}

/**
* @brief UART5 Initialization Function
* @param None
* @retval None
*/
static void MX_UART5_Init(void)
{
  
  /* USER CODE BEGIN UART5_Init 0 */
  
  /* USER CODE END UART5_Init 0 */
  
  /* USER CODE BEGIN UART5_Init 1 */
  
  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  
  /* USER CODE END UART5_Init 2 */
  
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  
  /* USER CODE END USART2_Init 2 */
  
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  
  /* USER CODE END USART3_Init 2 */
  
}

/**
* @brief USART6 Initialization Function
* @param None
* @retval None
*/
static void MX_USART6_UART_Init(void)
{
  
  /* USER CODE BEGIN USART6_Init 0 */
  
  /* USER CODE END USART6_Init 0 */
  
  /* USER CODE BEGIN USART6_Init 1 */
  
  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  
  /* USER CODE END USART6_Init 2 */
  
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STATE_LED0_Pin|STATE_LED1_Pin|STATE_LED2_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FAN1_ONOFF_Pin|FAN2_ONOFF_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIZNET_RESET_GPIO_Port, WIZNET_RESET_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, NBL0_Pin|NBL1_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : FAN_A2_SENSOR_Pin FAN_B1_SENSOR_Pin */
  GPIO_InitStruct.Pin = FAN_A2_SENSOR_Pin|FAN_B1_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STATE_LED0_Pin STATE_LED1_Pin STATE_LED2_Pin */
  GPIO_InitStruct.Pin = STATE_LED0_Pin|STATE_LED1_Pin|STATE_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STM32_GPS_PPS_Pin STM32_INS_PPS_Pin */
  GPIO_InitStruct.Pin = STM32_GPS_PPS_Pin|STM32_INS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FAN1_ONOFF_Pin FAN2_ONOFF_Pin */
  GPIO_InitStruct.Pin = FAN1_ONOFF_Pin|FAN2_ONOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pin : FAN_A1_SENSOR_Pin */
  GPIO_InitStruct.Pin = FAN_A1_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAN_A1_SENSOR_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : WIZNET_RESET_Pin */
  GPIO_InitStruct.Pin = WIZNET_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIZNET_RESET_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : FAN_B2_SENSOR_Pin */
  GPIO_InitStruct.Pin = FAN_B2_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAN_B2_SENSOR_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : NBL0_Pin NBL1_Pin */
  GPIO_InitStruct.Pin = NBL0_Pin|NBL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
  
  /* USER CODE BEGIN FSMC_Init 0 */
  
  /* USER CODE END FSMC_Init 0 */
  
  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  
  /* USER CODE BEGIN FSMC_Init 1 */
  
  /* USER CODE END FSMC_Init 1 */
  
  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FSMC_NORSRAM_DEVICE;
  hsram2.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FSMC_NORSRAM_BANK2;
  hsram2.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram2.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram2.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram2.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram2.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 0;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 3;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  
  if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }
  
  /* USER CODE BEGIN FSMC_Init 2 */
  
  /* USER CODE END FSMC_Init 2 */
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
