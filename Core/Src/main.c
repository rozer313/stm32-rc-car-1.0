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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t current_power = 0;

uint32_t distance_cm;
uint32_t captured_distances[3];
uint8_t number_of_measurements = 0;
uint8_t continue_measuring = 1;
uint8_t signal_stop = 0;
uint32_t last_distance = 0;
uint16_t no_same_distances = 0;
uint8_t is_stuck = 0;
char buffer[30];
int size;
uint16_t servo_angle_ms = 1500;

uint8_t signal_start = 0;

char *demo_string = "test\n\r"; //demo string
uint8_t recv_char;
uint8_t recv_str[20];
int i=0;
char *forward = "forward";
char *backwards = "backwards";
char *left = "left";
char *right = "right";
char *ble_stop = "stop";
uint8_t ft = 76;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t convert_to_cm(uint32_t distance_us)
{
	return (distance_us / 58);
}

/* HC-SR-04 AND SERVO */
uint32_t median_filter(uint32_t *array) {
	uint32_t temp;
	for (int i=0; i<2; i++) {
		for (int j=i+1; j<3; j++) {
			if (array[j] < array[i])
			{
				temp = array[j];
				array[j] = array[i];
				array[i] = temp;
			}
		}
	}
	return array[2];
}



void servo_scan_left() {
	servo_angle_ms += 10;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, servo_angle_ms);
	HAL_Delay(20);
}

void servo_scan_right() {
	servo_angle_ms -= 10;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, servo_angle_ms);
	HAL_Delay(20);
}

void servo_center() {
	servo_angle_ms = 1500;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, servo_angle_ms);
}

/* ENGINE CONTROL */

void accelerate(uint16_t speed) {
	current_power = 100;
	while (current_power < speed) {
		current_power += 10;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, current_power);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, current_power);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, current_power);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, current_power);
		HAL_Delay(30);
	}
	current_power = speed;
}

void move(uint16_t speed) {
	//if (current_power < speed)
	//	accelerate(speed);

	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, speed);
	current_power = speed;
}

void stop() {
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
	current_power = 0;
}

void turn_forward() {
	HAL_GPIO_WritePin(F_IN1_GPIO_Port, F_IN1_Pin, 1);
	HAL_GPIO_WritePin(F_IN2_GPIO_Port, F_IN2_Pin, 0);
	HAL_GPIO_WritePin(F_IN3_GPIO_Port, F_IN3_Pin, 0);
	HAL_GPIO_WritePin(F_IN4_GPIO_Port, F_IN4_Pin, 1);

	HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, 1);
	HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, 0);
	HAL_GPIO_WritePin(R_IN3_GPIO_Port, R_IN3_Pin, 0);
	HAL_GPIO_WritePin(R_IN4_GPIO_Port, R_IN4_Pin, 1);
}

void turn_backwards() {
	HAL_GPIO_WritePin(F_IN1_GPIO_Port, F_IN1_Pin, 0);
	HAL_GPIO_WritePin(F_IN2_GPIO_Port, F_IN2_Pin, 1);
	HAL_GPIO_WritePin(F_IN3_GPIO_Port, F_IN3_Pin, 1);
	HAL_GPIO_WritePin(F_IN4_GPIO_Port, F_IN4_Pin, 0);

	HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, 0);
	HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, 1);
	HAL_GPIO_WritePin(R_IN3_GPIO_Port, R_IN3_Pin, 1);
	HAL_GPIO_WritePin(R_IN4_GPIO_Port, R_IN4_Pin, 0);
}

void turn_left() {
	HAL_GPIO_WritePin(F_IN1_GPIO_Port, F_IN1_Pin, 0);
	HAL_GPIO_WritePin(F_IN2_GPIO_Port, F_IN2_Pin, 1);
	HAL_GPIO_WritePin(F_IN3_GPIO_Port, F_IN3_Pin, 0);
	HAL_GPIO_WritePin(F_IN4_GPIO_Port, F_IN4_Pin, 1);

	HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, 0);
	HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, 1);
	HAL_GPIO_WritePin(R_IN3_GPIO_Port, R_IN3_Pin, 0);
	HAL_GPIO_WritePin(R_IN4_GPIO_Port, R_IN4_Pin, 1);
}

void turn_right() {
	HAL_GPIO_WritePin(F_IN1_GPIO_Port, F_IN1_Pin, 1);
	HAL_GPIO_WritePin(F_IN2_GPIO_Port, F_IN2_Pin, 0);
	HAL_GPIO_WritePin(F_IN3_GPIO_Port, F_IN3_Pin, 1);
	HAL_GPIO_WritePin(F_IN4_GPIO_Port, F_IN4_Pin, 0);

	HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, 1);
	HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, 0);
	HAL_GPIO_WritePin(R_IN3_GPIO_Port, R_IN3_Pin, 1);
	HAL_GPIO_WritePin(R_IN4_GPIO_Port, R_IN4_Pin, 0);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(TIM1 == htim->Instance)
	{

		uint32_t echo_value;
		echo_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		echo_value = convert_to_cm(echo_value);

		captured_distances[number_of_measurements++] = echo_value;
		if(number_of_measurements > 2 && continue_measuring) {
			distance_cm = median_filter(captured_distances);
			number_of_measurements = 0;
			size = sprintf(buffer, "value=%lu\n\r", distance_cm);

			if (abs(last_distance - distance_cm) <= 25 && signal_start == 1) {
				no_same_distances++;
				if(no_same_distances >= 10) {
					signal_stop = 1;
				}

			}
			else {
				no_same_distances = 0;
			}

			//HAL_UART_Transmit(&huart2, (uint8_t*)&buffer, size, 500);
			if (distance_cm <= 30 && signal_start) {
				//continue_measuring = 0;
				signal_stop = 1;
				stop();
				//HAL_UART_Transmit(&huart2, "STOP\n\r", 6, 500);
			}
			last_distance = distance_cm;
		}
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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  uint16_t time = 1500;
  uint16_t power = 300;

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart1, &recv_char, 1);

  HAL_Delay(1000);
  uint8_t check_left = 1;
  uint32_t mean_value_left = 0;
  uint32_t mean_value_right = 0;
  uint32_t last_measured_value = 0;
  uint16_t measures_while_scanning = 0;
  uint16_t max_left_value = 0;
  uint16_t max_right_value = 0;
  uint16_t servo_left_max = 0;
  uint16_t servo_right_max = 0;
  //servo_scan_left();
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 1500);
  //turn_right();
  //move(450);
  HAL_Delay(1250);
  //stop();


  HAL_UART_Transmit(&huart2, "LEFT\n\r", 6, 500); //go left
  HAL_UART_Transmit(&huart1, (uint8_t*)demo_string, strlen(demo_string), HAL_MAX_DELAY);
  //servo_center();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Transmit(&huart1, (uint8_t*)demo_string, strlen(demo_string), HAL_MAX_DELAY);

	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
		  signal_start = 1;
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  }


	  if (signal_stop && signal_start) {
		  stop();
		  if (check_left == 1) {
			  if (servo_angle_ms < 2500) {
				  servo_scan_left();
				  if (last_measured_value != distance_cm)
				  {
					  last_measured_value = distance_cm;
					  if (last_measured_value > max_left_value) {
						  max_left_value = last_measured_value;
						  servo_left_max = servo_angle_ms;
					  }
					  mean_value_left += distance_cm;
					  measures_while_scanning++;
				  }
			  }
			  else {
				  mean_value_left = mean_value_left / measures_while_scanning;
				  measures_while_scanning = 0;
				  last_measured_value = 0;
				  check_left = 0;
				  servo_center();
			  }
		  }

		  else if (check_left == 0) {

			  if (servo_angle_ms > 500) {
				  servo_scan_right();
				  if (last_measured_value != distance_cm)
				  {
					  last_measured_value = distance_cm;
					  if (last_measured_value > max_right_value) {
						  max_right_value = last_measured_value;
						  servo_right_max = servo_angle_ms;
					  }
					  mean_value_right += distance_cm;
					  measures_while_scanning++;
				  }
			  }
			  else {
				  mean_value_right = mean_value_right / measures_while_scanning;
				  measures_while_scanning = 0;
				  last_measured_value = 0;
				  check_left = 2;
				  servo_center();
			  }
		  }

		  else {
			  turn_backwards();
			  move(300);
			  HAL_Delay(150);
			  if (mean_value_left > mean_value_right) {
				  HAL_UART_Transmit(&huart2, "LEFT\n\r", 6, 500); //go left
				  turn_left();
				  do {
					  move(450);
					  HAL_Delay(250);
				  } while (distance_cm < max_right_value && distance_cm < 40);
			  }

			  else {
				  HAL_UART_Transmit(&huart2, "RIGHT\n\r", 7, 500); //go right
				  turn_right();
				  do {
					  move(450);
					  HAL_Delay(250);

				  } while (distance_cm < max_right_value && distance_cm < 40);
			  }
			  stop();

			  turn_forward();

			  mean_value_right = 0;
			  mean_value_left = 0;
			  measures_while_scanning = 0;
			  last_measured_value = 0;
			  check_left = 1;
			  max_left_value = 0;
			  max_right_value = 0;
			  servo_left_max = 0;
			  servo_right_max = 0;
			  signal_stop = 0;

		  }

	  }

	  else if (signal_start){
		  turn_forward();
		  if (distance_cm < 40 && distance_cm > 25)
			  move(200);
		  else
			  move(power);
	  }



	  /*
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  turn_forward();
		  move(power);
		  HAL_Delay(time);
		  stop();
		  turn_backwards();
		  move(power);
		  HAL_Delay(time);
		  stop();
		  turn_left();
		  move(power);
		  HAL_Delay(time);
		  stop();
		  turn_right();
		  move(power);
		  HAL_Delay(time);
		  stop();

		  //WYKOMENTOWAC
		  turn_forward();
		  move(400);
		  HAL_Delay(2500);
		  stop();
		  turn_backwards();
		  move(400);
		  HAL_Delay(2500);
		  stop();


		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  } */


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 62499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R_IN1_Pin|F_IN4_Pin|R_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, F_IN1_Pin|F_IN2_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, F_IN3_Pin|R_IN3_Pin|R_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_IN1_Pin F_IN4_Pin R_IN4_Pin */
  GPIO_InitStruct.Pin = R_IN1_Pin|F_IN4_Pin|R_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : F_IN1_Pin F_IN2_Pin LD2_Pin */
  GPIO_InitStruct.Pin = F_IN1_Pin|F_IN2_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : F_IN3_Pin R_IN3_Pin R_IN2_Pin */
  GPIO_InitStruct.Pin = F_IN3_Pin|R_IN3_Pin|R_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1 ){
        if(recv_char == '\r'){

	    HAL_UART_Transmit(&huart2, recv_str, i, HAL_MAX_DELAY);

	    	if(!strcmp(recv_str, forward)) {
	    		turn_forward();
	    		move(350);
	    		//HAL_Delay(500);
	    		//stop();
	    	}

	    	if(!strcmp(recv_str, backwards)) {
	    		turn_backwards();
	    		move(350);
	    		//HAL_Delay(500);
	    		//stop();
	    	}

	    	if(!strcmp(recv_str, left)) {
	    		//sprintf(recv_str, "stop");
	    		turn_left();
	    		move(450);
	    		//HAL_Delay(500);
	    		//stop();
	    		//sprintf(recv_str, "stop");
	    	}

	    	if(!strcmp(recv_str, right)) {
	    		//sprintf(recv_str, "stop");
	    		turn_right();
	    		move(450);
	    		//HAL_Delay(500);
	    		//stop();

	    	}

	    	if(!strcmp(recv_str, ble_stop)) {
	    		stop();
	    		sprintf(recv_str, "stop");
	    	}


			memset(recv_str, 0, i);
			i=0;
		}
        else {
		    if(recv_char == '\r' || recv_char == '\n'){
		}
		    else{
		    recv_str[i++] = recv_char;
		}
	 }
	 HAL_UART_Receive_IT(&huart1, &recv_char, 1);

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
