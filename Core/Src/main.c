/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define only_activate_Motor 0
#define only_disable_Motor 0
#define only_activate_linMotor_Set 0
#define only_activate_linMotor_1 0
#define only_activate_linMotor_2 0
#define only_activate_linMotor_test_10mm 0
#define stage_disable 0
#define test 0
#define runway_disable 0

#define stage 0

#define backword_speed 0.65

typedef enum{
	forword=0,
	backword=1,
	left=2,
	right=3,
	middle=4
}motor_direction;
typedef enum{
	break_Release=1,
	break_on=0,
}motor_break;

typedef enum{
	basic_0=0,
	basic_1,
	standing,
	sit
}standingStep;


typedef struct{
	int16_t e16b_new;
	int16_t e16b_diff;
	int16_t e16b_old;
	int32_t e32b_new;
	int32_t e32b_diff;
	int32_t e32b_old;
}encoder;

typedef struct{
	uint8_t en1;
	uint8_t en2;
	uint8_t en3;
	uint8_t en4;
	uint8_t en5;
}flag_encoder;

typedef struct{
	int32_t t1;
	int32_t t2;
	int32_t t3;
	int32_t t4;
	int32_t t5;

	int32_t t1Diff;
	int32_t t2Diff;
	int32_t t3Diff;
	int32_t t4Diff;
	int32_t t5Diff;

	int32_t t2_1Diff;
	int32_t t2_3Diff;
	int32_t t2_4Diff;
	int32_t t2_5Diff;

	int32_t t3_4Diff;

}target_encoder;


typedef struct{
	uint8_t ln1;
	uint8_t ln2;
	uint8_t ln3;
	uint8_t ln4;
	uint8_t ln5;

}activate;

typedef enum{
	CW=1,
	CCW=0
}linMotorDir;

typedef enum{
	status_Stage_PULL,
	status_Stage_PUSH
};

typedef enum{
	status_runway_UP,
	status_runway_Down
};


#define Default_Lin_Stage_Movement_Time 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
double  speed=1.0;
uint16_t joy_x, joy_y;
motor_direction dir_motor_LR,dir_motor_FB;
activate active={0,0,0,0,0};

int32_t pwm_L=0,PWM_R=0,n_pwm_L=0,n_PWM_R=0,pre_pwm_L=0,pre_PWM_R=0;

//forword == 1 backword ==2
uint16_t motor_dir_flag=0;

uint8_t seat=0;

flag_encoder encoder_flag;
target_encoder target;
encoder encoder1,encoder2,encoder3,encoder4,encoder5;

uint8_t walker_flag;
uint8_t default_set_flag;
uint8_t standing_complete_flag;
uint16_t delay_sec=0;
uint16_t delay_sec_stage=0;


uint8_t main_loop_on;
uint8_t idle_pwm=75;

uint8_t stage_status=9;
uint8_t runway_status=9;
uint8_t stage_delay_act=0;

uint8_t  SET_Lin_Stage_Movement_Time=0;
uint8_t  SET_Lin_Stage_Movement_Cali_PULL=3;

uint8_t SET_Lin_Stage_PULL = 18;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
void readEncoder();
void calEncoder(uint8_t function);
void enable(uint8_t sel, uint8_t upOrDown);
void lin_reset();
void encoder_reset();

void lin_motor_Up(uint8_t pos);
void lin_motor_Down(uint8_t pos);
void lin_motor_init_en();
int32_t mmToPulse(int32_t mm);


void runway_Up();
void runway_Down();

void walker_hold();
void walker_release();

void stage_PUSH();
void stage_PULL();

void new_standing_method(uint8_t step);
void new_pwm_set(uint8_t lin1,uint8_t lin2,uint8_t lin3,uint8_t lin4,uint8_t lin5);

void new_pwm_stop(uint8_t sel);
void new_pwm_en(uint8_t sel);
void new_delay_cal(float mm,float Load, float pwm);
void calibration_4();

void set();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)								/* for printf */
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE			// For printf Function
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);   // USB
	return ch;
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
	MX_ADC1_Init();
	MX_USART3_UART_Init();
	MX_TIM11_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM12_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_TIM10_Init();
	MX_TIM13_Init();
	MX_TIM7_Init();
	MX_TIM6_Init();
	MX_TIM14_Init();
	MX_TIM9_Init();
	/* USER CODE BEGIN 2 */
	//L_motor = tim12_CH2 R_motor = tim 11_CH1

	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	HAL_TIM_Base_Start_IT(&htim6);


	//lim_base count 1sec
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1|TIM_CHANNEL_2);

	main_loop_on=0;
	motor_dir_flag=0;
	uint8_t unknown_flag=0;
	uint8_t flag_stand_first=0;
	uint8_t cali_flag=0;
	uint8_t test_alpha =0;


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_Delay(10000);

	while (1)
	{

#if stage
#endif

#if !stage

#if !test
#if only_activate_linMotor_test_10mm


#endif
#if !only_activate_linMotor_test_10mm

#if	!only_activate_linMotor_Set

#if !only_activate_Motor
		if(HAL_GPIO_ReadPin(state_controller_GPIO_Port, state_controller_Pin)==GPIO_PIN_SET ){
			main_loop_on=1;
			cali_flag=0;

		}
		else{
			main_loop_on=0;
			if(!cali_flag){
				runway_Up();
				stage_PUSH();
				calibration_4();
				cali_flag=1;
			}
		}
		if(main_loop_on){
#endif
			if(n_pwm_L ==0 && n_PWM_R == 0){
				if(HAL_GPIO_ReadPin(motor_speed_up_GPIO_Port, motor_speed_up_Pin)==GPIO_PIN_SET ){
					speed=3.0;

				}
				else if(HAL_GPIO_ReadPin(motor_speed_down_GPIO_Port, motor_speed_down_Pin)==GPIO_PIN_SET ){
					speed=1.0;
				}
				else{
					speed=2.0;
				}

				if(speed == 1){
					HAL_GPIO_WritePin(share_speed_GPIO_Port, share_speed_Pin, SET);
					HAL_GPIO_WritePin(share_speed_1_GPIO_Port, share_speed_1_Pin, RESET);

				}

				else if(speed == 2){
					HAL_GPIO_WritePin(share_speed_GPIO_Port, share_speed_Pin, RESET);
					HAL_GPIO_WritePin(share_speed_1_GPIO_Port, share_speed_1_Pin, SET);

				}

				else if(speed == 3){
					HAL_GPIO_WritePin(share_speed_GPIO_Port, share_speed_Pin, SET);
					HAL_GPIO_WritePin(share_speed_1_GPIO_Port, share_speed_1_Pin, SET);
				}
			}
#endif
#if !only_activate_Motor

#if	!only_activate_linMotor_Set
			if(HAL_GPIO_ReadPin(share_default_set_GPIO_Port, share_default_set_Pin)==GPIO_PIN_SET){
#endif
#if !only_activate_linMotor_2
				stage_PULL();
				new_standing_method(standing);

				walker_flag=0;
				delay_sec=0;
				default_set_flag=0;

				runway_Down();
				walker_release();
				while(!walker_flag){
#if	!only_activate_linMotor_Set

					if(HAL_GPIO_ReadPin(Walker_in_GPIO_Port, Walker_in_Pin)==GPIO_PIN_SET){
						HAL_Delay(250);
						walker_hold();
						runway_Up();
#endif
						walker_flag=1;
						default_set_flag=1;

#if	!only_activate_linMotor_Set
					}
#endif

					//What if the walker is not recognized? -> Seat position initialization
					//					if(delay_sec>=120){
					//						walker_flag=1;
					//						default_set_flag=0;
					//					}
				}
				HAL_Delay(5000);
				stage_PUSH();

				new_standing_method(sit);

				//				act time = ?????????
				if(default_set_flag){

					HAL_GPIO_WritePin(share_default_state_GPIO_Port, share_default_state_Pin, SET);
					delay_sec=0;
					//control board recognize time _5sec
					while(delay_sec<5);
					HAL_GPIO_WritePin(share_default_state_GPIO_Port, share_default_state_Pin, RESET);

				}

#endif
#if	!only_activate_linMotor_Set
			}
#endif
#endif
#if	!only_activate_linMotor_Set

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 200);
			while((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) != HAL_ADC_STATE_REG_EOC);
			joy_x=HAL_ADC_GetValue(&hadc1);

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 200);
			while((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) != HAL_ADC_STATE_REG_EOC);
			joy_y=HAL_ADC_GetValue(&hadc1);


			if(joy_x<800)	dir_motor_LR=right;
			else if(joy_x>3200)	dir_motor_LR=left;
			else	dir_motor_LR=middle;
			if(joy_y>3400)	dir_motor_FB=backword;
			else if(joy_y<1000)	dir_motor_FB=forword;
			else dir_motor_FB=middle;
#endif

#if !only_activate_Motor
#if	!only_activate_linMotor_Set

			if(n_PWM_R ==0 && n_pwm_L ==0){
				HAL_GPIO_WritePin(share_stop_out_GPIO_Port, share_stop_out_Pin, SET);
			}
			else{
				HAL_GPIO_WritePin(share_stop_out_GPIO_Port, share_stop_out_Pin, RESET);
			}


			if(HAL_GPIO_ReadPin(share_seat_in_GPIO_Port,share_seat_in_Pin) == GPIO_PIN_SET){
				seat=1;
				unknown_flag=1;
			}
			else{
				if(n_PWM_R ==0 && n_pwm_L ==0){
					seat=0;
					unknown_flag=0;
					if(dir_motor_FB==middle &&  dir_motor_LR==middle){
						HAL_GPIO_WritePin(share_seat_joy_out_GPIO_Port, share_seat_joy_out_Pin, SET);
					}
					else{
						HAL_GPIO_WritePin(share_seat_joy_out_GPIO_Port, share_seat_joy_out_Pin, RESET);
					}
				}
			}
#endif

#if	!only_activate_linMotor_Set
			///////////////////////////////////////////////////////////////////////////////////////
			if(HAL_GPIO_ReadPin(share_standing_method_GPIO_Port, share_standing_method_Pin)==GPIO_PIN_SET){
#endif
#if !only_activate_linMotor_1

				stage_PULL();

				walker_flag=0;
				delay_sec=0;
				standing_complete_flag=0;

				new_standing_method(standing);

				runway_Down();
				walker_release();


				while(!walker_flag){
#if	!only_activate_linMotor_Set
					if(HAL_GPIO_ReadPin(Walker_in_GPIO_Port, Walker_in_Pin)==GPIO_PIN_RESET){
#endif
						walker_flag=1;
						standing_complete_flag=1;

#if	!only_activate_linMotor_Set
					}
					else{
						walker_flag=0;
					}

#endif
				}

				delay_sec=0;
				if(standing_complete_flag == 1){
					HAL_GPIO_WritePin(share_standing_complete_GPIO_Port, share_standing_complete_Pin, SET);
					//control board recognize time _5sec
					while(delay_sec<10);
					HAL_GPIO_WritePin(share_standing_complete_GPIO_Port, share_standing_complete_Pin, RESET);
					runway_Up();
					HAL_Delay(10000);
				}
#endif
#if	!only_activate_linMotor_Set

			}
#endif

#endif
#if only_activate_Motor
			unknown_flag=1;
#endif
#if	!only_activate_linMotor_Set

			if(unknown_flag==1){
#if !only_activate_Motor
				HAL_GPIO_WritePin(share_seat_joy_out_GPIO_Port, share_seat_joy_out_Pin, SET);
#endif
				//motor control
				if(dir_motor_FB == forword){
					motor_dir_flag=1;
					if(dir_motor_LR == middle){
						pwm_L=100;
						PWM_R=100;
					}
					else if(dir_motor_LR == left){
						pwm_L=100-20;
						PWM_R=100;
					}
					else if(dir_motor_LR == right){
						pwm_L=100;
						PWM_R=100-20;
					}
				}
				else if(dir_motor_FB == backword){
					motor_dir_flag=2;

					if(dir_motor_LR == middle){
						pwm_L=-100.0*backword_speed;
						PWM_R=-100.0*backword_speed;
					}
					else if(dir_motor_LR == left){
						pwm_L=-(100.0-20.0)*backword_speed;
						PWM_R=-100.0*backword_speed;
					}
					else if(dir_motor_LR == right){
						pwm_L=-100.0*backword_speed;
						PWM_R=-(100.0-20.0)*backword_speed;
					}
				}
				else if(dir_motor_FB == middle){

					if(dir_motor_LR == middle){
						motor_dir_flag=3;
						pwm_L=pre_pwm_L;
						PWM_R=pre_PWM_R;
					}
					else if(dir_motor_LR == left){
						motor_dir_flag=1;
						pwm_L=40;
						PWM_R=80;
					}
					else if(dir_motor_LR == right){
						motor_dir_flag=1;
						pwm_L=80;
						PWM_R=40;
					}
				}
				if(motor_dir_flag==1 || motor_dir_flag==2)
				{
					pre_PWM_R=PWM_R;
					pre_pwm_L=pwm_L;
				}


#if !only_disable_Motor

				pwm_L = (double)pwm_L*(double)speed/3.0;
				//				if(PWM_R > 0)
				PWM_R = (double)PWM_R*(double)speed/3.0;
				//				else if(PWM_R < 0)
				//					PWM_R = (double)PWM_R*(speed/3.0);
				//				else
				//					PWM_R=0;
				//
				//				if(pwm_L > 0)
				//					pwm_L = (double)pwm_L*(speed/3.0)+4.0;
				//				else if(pwm_L < 0)
				//					pwm_L = (double)pwm_L*(speed/3.0)-20.0;
				//				else
				//					pwm_L=0;

#endif

#if only_disable_Motor
				pwm_L=0;
				PWM_R=0;
#endif


				if(dir_motor_LR == right){
					HAL_GPIO_WritePin(share_Right_GPIO_Port, share_Right_Pin, SET);
				}
				else{
					HAL_GPIO_WritePin(share_Right_GPIO_Port, share_Right_Pin, RESET);
				}
				if(dir_motor_LR == left){
					HAL_GPIO_WritePin(share_Left_GPIO_Port, share_Left_Pin, SET);
				}
				else{
					HAL_GPIO_WritePin(share_Left_GPIO_Port, share_Left_Pin, RESET);
				}
				if((n_PWM_R<=0 || n_pwm_L<=0) && dir_motor_FB == backword){
					HAL_GPIO_WritePin(share_Back_GPIO_Port, share_Back_Pin, SET);
				}
				else{
					HAL_GPIO_WritePin(share_Back_GPIO_Port, share_Back_Pin, RESET);
				}
				//				printf("x:%d y:%d \r\n",joy_x,joy_y);
				//				printf("x:%d y:%d \r\n",dir_motor_LR,dir_motor_FB);
				//				printf("pwm_L:%d PWM_R:%d \r\n",pwm_L,PWM_R);
				//				printf("speed:%d \r\n",speed);
				//				printf("n_R:%d n_L:%d \r\n",n_PWM_R,n_pwm_L);

				HAL_Delay(100);
			}
#if !only_activate_Motor
		}
#endif
#endif
#if only_activate_linMotor_Set
		break;
#endif
#endif
#endif
#endif
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
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

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 42-1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 100-1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

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
	htim6.Init.Prescaler = 8400-1;
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
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 8400-1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 10000-1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 65535;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void)
{

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 84-1;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 100-1;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */
	HAL_TIM_MspPostInit(&htim9);

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void)
{

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 84-1;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 100-1;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */
	HAL_TIM_MspPostInit(&htim10);

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void)
{

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 84-1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 100-1;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */
	HAL_TIM_MspPostInit(&htim11);

}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void)
{

	/* USER CODE BEGIN TIM12_Init 0 */

	/* USER CODE END TIM12_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM12_Init 1 */

	/* USER CODE END TIM12_Init 1 */
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 42-1;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = 100-1;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM12_Init 2 */

	/* USER CODE END TIM12_Init 2 */
	HAL_TIM_MspPostInit(&htim12);

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
	htim13.Init.Prescaler = 42-1;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 100-1;
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
	sConfigOC.Pulse = 0;
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
	htim14.Init.Prescaler = 1280-1;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 100-1;
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
	sConfigOC.Pulse = 20;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, motor_L_DIR_Pin|motor_L_break_Pin|motor_R_DIR_Pin|share_Left_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, DIR_linear_4_Pin|DIR_linear_5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, share_seat_joy_out_Pin|motor_R_break_Pin|share_default_state_Pin|share_standing_complete_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, DIR_linear_3_Pin|share_speed_Pin|lin_Wheel_Dir_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, share_Right_Pin|share_Back_Pin|LD2_Pin|share_stop_out_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, runway_in1_Pin|runway_in2_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, runway1_in1_Pin|runway1_in2_Pin|Walker_motor_in1_Pin|lin_Wheel_En_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Walker_motor_in2_GPIO_Port, Walker_motor_in2_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, DIR_linear_1_Pin|DIR_linear_2_Pin|share_speed_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : motor_L_DIR_Pin motor_L_break_Pin motor_R_DIR_Pin share_Left_Pin */
	GPIO_InitStruct.Pin = motor_L_DIR_Pin|motor_L_break_Pin|motor_R_DIR_Pin|share_Left_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_linear_4_Pin DIR_linear_5_Pin */
	GPIO_InitStruct.Pin = DIR_linear_4_Pin|DIR_linear_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : motor_speed_up_Pin motor_speed_down_Pin share_standing_method_Pin */
	GPIO_InitStruct.Pin = motor_speed_up_Pin|motor_speed_down_Pin|share_standing_method_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : share_seat_joy_out_Pin motor_R_break_Pin share_default_state_Pin share_standing_complete_Pin */
	GPIO_InitStruct.Pin = share_seat_joy_out_Pin|motor_R_break_Pin|share_default_state_Pin|share_standing_complete_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : share_seat_in_Pin share_default_set_Pin Walker_in_Pin */
	GPIO_InitStruct.Pin = share_seat_in_Pin|share_default_set_Pin|Walker_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_linear_3_Pin share_speed_Pin Walker_motor_in2_Pin lin_Wheel_Dir_Pin */
	GPIO_InitStruct.Pin = DIR_linear_3_Pin|share_speed_Pin|Walker_motor_in2_Pin|lin_Wheel_Dir_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : stage_PULL_SW_Pin */
	GPIO_InitStruct.Pin = stage_PULL_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(stage_PULL_SW_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : share_Right_Pin share_Back_Pin runway_in1_Pin runway_in2_Pin
                           LD2_Pin share_stop_out_Pin */
	GPIO_InitStruct.Pin = share_Right_Pin|share_Back_Pin|runway_in1_Pin|runway_in2_Pin
			|LD2_Pin|share_stop_out_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : runway1_in1_Pin runway1_in2_Pin DIR_linear_1_Pin DIR_linear_2_Pin
                           Walker_motor_in1_Pin share_speed_1_Pin lin_Wheel_En_Pin */
	GPIO_InitStruct.Pin = runway1_in1_Pin|runway1_in2_Pin|DIR_linear_1_Pin|DIR_linear_2_Pin
			|Walker_motor_in1_Pin|share_speed_1_Pin|lin_Wheel_En_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : default_Pin state_controller_Pin */
	GPIO_InitStruct.Pin = default_Pin|state_controller_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t limit=0;
//0.1sec_loop *20 ==2sec_loop
uint16_t timing_loop_Start_L,timing_loop_Start_R,timing_loop_Stop_L,timing_loop_Stop_R,changing_timing_loop=0;
uint16_t C_pwm_L=0,C_pwm_R=0;


uint8_t any_stop_flag_TIM=0;
float loop_stop_constant = 5.0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance ==TIM7){
		delay_sec+=1;
		if(stage_delay_act){
			delay_sec_stage+=1;
			if(stage_status == status_Stage_PULL){
				SET_Lin_Stage_Movement_Time=SET_Lin_Stage_PULL;
			}
			else if(stage_status == status_Stage_PUSH){
				SET_Lin_Stage_Movement_Time=Default_Lin_Stage_Movement_Time;
			}

			if(delay_sec_stage>=SET_Lin_Stage_Movement_Time){
				HAL_GPIO_WritePin(lin_Wheel_En_GPIO_Port, lin_Wheel_En_Pin, SET);
				HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
				stage_delay_act=0;
			}
		}
		else{
			delay_sec_stage=0;
			__HAL_TIM_SET_COUNTER(&htim14,0);
		}
	}


	if(htim->Instance == TIM6){
		if(pwm_L<0){
			timing_loop_Start_L=-pwm_L/100.0*30.0;
			timing_loop_Stop_L=-pwm_L/100.0*20.0;
		}
		else{
			timing_loop_Start_L=pwm_L/100.0*30.0;
			timing_loop_Stop_L=pwm_L/100.0*20.0;

		}
		if(PWM_R<0){
			timing_loop_Start_R=-PWM_R/100.0*30.0;
			timing_loop_Stop_R=-PWM_R/100.0*20.0;
		}
		else{
			timing_loop_Start_R=PWM_R/100.0*30.0;
			timing_loop_Stop_R=PWM_R/100.0*20.0;
		}




		if(timing_loop_Start_L>=timing_loop_Start_R){
			if(timing_loop_Start_L<=timing_loop_Start_R+10)
				timing_loop_Start_R=timing_loop_Start_L;
		}
		else{
			if(timing_loop_Start_R<=timing_loop_Start_L+10)
				timing_loop_Start_L=timing_loop_Start_R;
		}

		if(timing_loop_Stop_L>=timing_loop_Stop_R){
			if(timing_loop_Stop_L<=timing_loop_Stop_R+7)
				timing_loop_Stop_R=timing_loop_Stop_L;
		}
		else{
			if(timing_loop_Stop_R<=timing_loop_Stop_L+7)
				timing_loop_Stop_L=timing_loop_Stop_R;
		}

		if(timing_loop_Start_L == 0) timing_loop_Start_L=1;
		if(timing_loop_Start_R == 0) timing_loop_Start_R=1;
		if(timing_loop_Stop_L == 0) timing_loop_Stop_L=1;
		if(timing_loop_Stop_R == 0) timing_loop_Stop_R=1;

		if(motor_dir_flag == 3 && (n_pwm_L == 0 || n_PWM_R == 0)){
			if(n_pwm_L ==0){
				timing_loop_Stop_R = 5;
			}
			else{
				timing_loop_Stop_L = 5;
			}
		}



		/*		//?���??????????????????????�?????????????????????? 미완?��.
		if(any_stop_flag_TIM){
			if(dir_motor_FB ==middle && dir_motor_LR==middle)
			{
				if(n_pwm_L ==0 || n_PWM_R ==0){
					if(n_pwm_L == 0 && n_PWM_R ==0){
						any_stop_flag_TIM=0;
					}
					else if(n_pwm_L ==0){
						timing_loop_Stop_R=(double)n_PWM_R/loop_stop_constant;
					}
					else{
						timing_loop_Stop_L=(double)n_pwm_L/loop_stop_constant;
					}
				}
			}
		}
		else{
			timing_loop_Stop_L=pwm_L/10.0*2.0;
			timing_loop_Stop_R=PWM_R/10.0*2.0;
		}*/



		if(motor_dir_flag==1){
			if(n_pwm_L<pwm_L){
				n_pwm_L +=pwm_L/timing_loop_Start_L;

			}

			if(n_pwm_L>pwm_L){
				if(pwm_L<0)
					timing_loop_Start_L=timing_loop_Start_L*2.0;
				n_pwm_L -=pwm_L/timing_loop_Start_L;

			}
			if(n_PWM_R<PWM_R){
				n_PWM_R +=PWM_R/timing_loop_Start_R;
			}
			if(n_PWM_R>PWM_R){
				if(PWM_R<0)
					timing_loop_Start_R=timing_loop_Start_R*2.0;
				n_PWM_R -=PWM_R/timing_loop_Start_R;
			}
		}
		if(motor_dir_flag==2){
			if(n_pwm_L<pwm_L){
				n_pwm_L -=pwm_L/timing_loop_Start_L;

			}
			if(n_pwm_L>pwm_L){
				if(pwm_L>0)
					timing_loop_Start_L=timing_loop_Start_L*2.0;
				n_pwm_L +=(pwm_L/timing_loop_Start_L);

			}
			if(n_PWM_R<PWM_R){
				n_PWM_R -=PWM_R/timing_loop_Start_R;
			}

			if(n_PWM_R>PWM_R){
				if(PWM_R>0)
					timing_loop_Start_R=timing_loop_Start_R*2.0;
				n_PWM_R +=PWM_R/timing_loop_Start_R;
			}
		}


		if(motor_dir_flag==3){

			if(pwm_L>=0){
				if(n_pwm_L>=0){
					if(n_pwm_L>10){
						n_pwm_L -= pwm_L/timing_loop_Stop_L;
					}
					else{
						n_pwm_L=0;
					}
				}
				else{
					if(n_pwm_L<-10){
						n_pwm_L += pwm_L/timing_loop_Stop_L;
					}
					else{
						n_pwm_L=0;
					}
				}
			}
			else{
				if(n_pwm_L>=0){
					if(n_pwm_L>10){
						n_pwm_L += pwm_L/timing_loop_Stop_L;
					}
					else{
						n_pwm_L=0;
					}
				}
				else{
					if(n_pwm_L<-10){
						n_pwm_L -= pwm_L/timing_loop_Stop_L;
					}
					else{
						n_pwm_L=0;
					}
				}
			}
			if(PWM_R>=0){
				if(n_PWM_R>=0){
					if(n_PWM_R>10){
						n_PWM_R -= PWM_R/timing_loop_Stop_R;
					}
					else{
						n_PWM_R=0;
					}
				}
				else{
					if(n_PWM_R<-10){
						n_PWM_R += PWM_R/timing_loop_Stop_R;
					}
					else{
						n_PWM_R=0;
					}
				}
			}
			else{
				if(n_PWM_R>=0){
					if(n_PWM_R>10){
						n_PWM_R += PWM_R/timing_loop_Stop_R;
					}
					else{
						n_PWM_R=0;
					}
				}
				else{
					if(n_PWM_R<-10){
						n_PWM_R -= PWM_R/timing_loop_Stop_R;
					}
					else{
						n_PWM_R=0;
					}
				}
			}
		}

		if(speed ==1){
			limit=50;
		}
		else if(speed ==2){
			limit=80;
		}
		else{
			limit=110;
		}
		if(n_PWM_R>limit || n_pwm_L>limit || n_pwm_L<-limit || n_PWM_R<-limit){
			n_PWM_R=0;
			n_pwm_L=0;
			motor_dir_flag=3;
		}
		else{
			if(n_PWM_R == 0 && n_pwm_L == 0){
				HAL_GPIO_WritePin(motor_L_break_GPIO_Port,motor_L_break_Pin, break_on);
				HAL_GPIO_WritePin(motor_R_break_GPIO_Port,motor_R_break_Pin, break_on);
			}
			else{
				HAL_GPIO_WritePin(motor_L_break_GPIO_Port,motor_L_break_Pin, break_Release);
				HAL_GPIO_WritePin(motor_R_break_GPIO_Port,motor_R_break_Pin, break_Release);
			}

			if(n_pwm_L<0){
				HAL_GPIO_WritePin(motor_L_DIR_GPIO_Port, motor_L_DIR_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,-n_pwm_L);
			}
			else if(n_pwm_L==0){
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
				__HAL_TIM_SET_COUNTER(&htim12,1);

			}
			else{
				HAL_GPIO_WritePin(motor_L_DIR_GPIO_Port, motor_L_DIR_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,n_pwm_L);
			}
			if(n_PWM_R<0){
				HAL_GPIO_WritePin(motor_R_DIR_GPIO_Port, motor_R_DIR_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,-n_PWM_R);
			}
			else if(n_PWM_R==0){
				__HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,0);
				__HAL_TIM_SET_COUNTER(&htim11,1);
			}
			else{
				HAL_GPIO_WritePin(motor_R_DIR_GPIO_Port, motor_R_DIR_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,n_PWM_R);
			}
		}
	}
}


void readEncoder(){
	encoder1.e16b_new=__HAL_TIM_GET_COUNTER(&htim1);
	HAL_Delay(50);
	encoder2.e16b_new=__HAL_TIM_GET_COUNTER(&htim2);
	HAL_Delay(50);
	encoder3.e16b_new=__HAL_TIM_GET_COUNTER(&htim3);
	HAL_Delay(50);
	encoder4.e16b_new=__HAL_TIM_GET_COUNTER(&htim4);
	HAL_Delay(50);
	encoder5.e16b_new=__HAL_TIM_GET_COUNTER(&htim8);
	HAL_Delay(50);

	encoder1.e16b_diff=encoder1.e16b_new-encoder1.e16b_old;
	encoder1.e16b_old=encoder1.e16b_new;
	encoder1.e32b_new+=encoder1.e16b_diff;

	encoder2.e16b_diff=encoder2.e16b_new-encoder2.e16b_old;
	encoder2.e16b_old=encoder2.e16b_new;
	encoder2.e32b_new+=encoder2.e16b_diff;
	encoder2.e32b_diff=encoder2.e32b_new-encoder2.e32b_old;
	encoder2.e32b_old=encoder2.e32b_new;

	encoder3.e16b_diff=encoder3.e16b_new-encoder3.e16b_old;
	encoder3.e16b_old=encoder3.e16b_new;
	encoder3.e32b_new+=encoder3.e16b_diff;

	encoder4.e16b_diff=encoder4.e16b_new-encoder4.e16b_old;
	encoder4.e16b_old=encoder4.e16b_new;
	encoder4.e32b_new+=encoder4.e16b_diff;

	encoder5.e16b_diff=encoder5.e16b_new-encoder5.e16b_old;
	encoder5.e16b_old=encoder5.e16b_new;
	encoder5.e32b_new+=encoder5.e16b_diff;

	target.t1Diff = target.t1-encoder1.e32b_new;
	target.t2Diff = target.t2-encoder2.e32b_new;
	target.t3Diff = target.t3-encoder3.e32b_new;
	target.t4Diff = target.t4-encoder4.e32b_new;
	target.t5Diff = target.t5-encoder5.e32b_new;

	target.t2_1Diff=target.t2Diff-target.t1Diff;
	target.t2_3Diff=target.t2Diff-target.t3Diff;
	target.t2_4Diff=target.t2Diff-target.t4Diff;
	target.t2_5Diff=target.t2Diff-target.t5Diff;

	target.t3_4Diff=target.t3Diff-target.t4Diff;
}

uint16_t DiffConstant = 30;
uint8_t pwm_adj=2;
uint8_t max_pwm=100;
uint8_t encoder_loop_flag=0;

//PWM_lin.pwm2=max_pwm*0.75;

//function 1: default //function2: seat_default //function3: standing
void calEncoder(uint8_t function){

	HAL_GPIO_WritePin(motor_R_break_GPIO_Port, motor_R_break_Pin, break_on);
	HAL_GPIO_WritePin(motor_L_break_GPIO_Port, motor_L_break_Pin, break_on);

	HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);

	switch (function){

	case 1:

		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t1=-65000;
		target.t2=-65000;
		target.t3=-65000;
		target.t4=-65000;

		target.t5=0;

		lin_motor_Down(9);

		while(encoder_loop_flag){


			readEncoder();
			enable(9,0);

			//down
			active.ln1=active.ln2=active.ln3=active.ln4=active.ln5=1;

			target.t5+=encoder2.e16b_diff;
			if(encoder_flag.en2 == 1){
				if(target.t2_1Diff>DiffConstant){
					active.ln2=0;

				}

				else if(target.t2_1Diff<-DiffConstant){
					active.ln1=0;

				}
				else{
					active.ln1=1;
					active.ln2=1;
				}
			}
			else{
				active.ln2=0;
				active.ln1=1;
			}


			if(target.t2_3Diff>DiffConstant){
				active.ln2=0;
			}

			else if(target.t2_3Diff<-DiffConstant){
				active.ln3=0;

			}
			else{
				active.ln2=1;
				active.ln3=1;
			}

			if(target.t2_4Diff>DiffConstant){
				active.ln2=0;
			}
			else if(target.t2_4Diff<-DiffConstant){
				active.ln4=0;
			}
			else{
				active.ln2=1;
				active.ln4=1;
			}

			set();

			//			diff16b == 0 -> stop
			lin_motor_init_en();

			if(encoder_flag.en1 == 0 && encoder_flag.en2 == 0 && encoder_flag.en3 == 0 && encoder_flag.en4 == 0 && encoder_flag.en5 == 0){
				encoder_loop_flag=0;
			}
		}

		break;

		//리니?�� 모터 1�?????????????????????? ?��?��(lin1 motor up)
	case 2:



		//step 1
		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t1=mmToPulse(25);
		target.t2=mmToPulse(25);

		lin_motor_Up(1);
		lin_motor_Up(2);
		while(encoder_loop_flag){

			readEncoder();
			enable(1,1);
			enable(2,1);

			if(target.t2_1Diff>DiffConstant){


			}

			else if(target.t2_1Diff<-DiffConstant){


			}
			else{


			}


			if(encoder_flag.en1 == 0 && encoder_flag.en2 == 0){
				encoder_loop_flag=0;
			}
		}

		//step 1
		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t1=mmToPulse(50);

		lin_motor_Up(1);
		while(encoder_loop_flag){

			readEncoder();
			enable(1,1);


			if(encoder_flag.en1 == 0){
				encoder_loop_flag=0;
			}
		}


		//step2
		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t1=mmToPulse(100);
		target.t2=mmToPulse(100);
		target.t3=mmToPulse(100);
		target.t4=mmToPulse(100);

		target.t5=0;

		lin_motor_Up(9);
		while(encoder_loop_flag){

			readEncoder();
			enable(9,1);

			//down

			target.t5+=encoder2.e32b_diff;
			if(target.t2_1Diff>DiffConstant){


			}

			else if(target.t2_1Diff<-DiffConstant){


			}
			else{


			}

			if(target.t2_3Diff>DiffConstant){


			}

			else if(target.t2_3Diff<-DiffConstant){


			}
			else{



			}
			if(target.t2_4Diff>DiffConstant){


			}

			else if(target.t2_4Diff<-DiffConstant){

			}
			else{


			}


			if(encoder_flag.en1 == 0 && encoder_flag.en2 == 0 && encoder_flag.en3 == 0 && encoder_flag.en4 == 0 && encoder_flag.en5 == 0){
				encoder_loop_flag=0;
			}
		}
		break;



		//standing
	case 3:

		//		미완?�� 그냥 ?��?���?????????????????????? 갖춰?�� �??????????????????????(?��?�� ?��?�� ?�� 코드?�� 비교)


		//////////////////////////////////////////////////////////////////////////
		//step 1

		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t1=-mmToPulse(125);
		target.t2=-mmToPulse(125);
		target.t3=-mmToPulse(125);
		target.t4=-mmToPulse(125);

		target.t5=0;

		lin_motor_Down(9);
		while(encoder_loop_flag){

			readEncoder();
			enable(9,0);

			//down

			target.t5+=encoder2.e32b_diff;
			if(target.t2_1Diff>DiffConstant){


			}

			else if(target.t2_1Diff<-DiffConstant){


			}
			else{


			}

			if(target.t2_3Diff>DiffConstant){


			}

			else if(target.t2_3Diff<-DiffConstant){


			}
			else{


			}
			if(target.t2_4Diff>DiffConstant){


			}

			else if(target.t2_4Diff<-DiffConstant){


			}
			else{

			}


			if(encoder_flag.en1 == 0 && encoder_flag.en2 == 0 && encoder_flag.en3 == 0 && encoder_flag.en4 == 0 && encoder_flag.en5 == 0){
				encoder_loop_flag=0;
			}
		}

		//////////////////////////////////////////////////////////////////////////
		//step 2

		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t1=-mmToPulse(50);

		lin_motor_Down(1);
		while(encoder_loop_flag){

			readEncoder();
			enable(1,0);

			if(encoder_flag.en1 == 0){
				encoder_loop_flag=0;
			}
		}

		//////////////////////////////////////////////////////////////////////////
		//step 3

		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();

		target.t3=mmToPulse(25);
		target.t4=mmToPulse(25);
		target.t5=0;

		lin_motor_Up(3);
		lin_motor_Up(4);
		lin_motor_Up(5);


		while(encoder_loop_flag){

			readEncoder();
			enable(3,1);
			enable(4,1);

			enable(5,1);

			//down

			target.t5+=encoder3.e16b_diff;

			if(target.t3_4Diff>DiffConstant){


			}

			else if(target.t3_4Diff<-DiffConstant){


			}
			else{


			}

			//			diff16b == 0 -> stop

			if(encoder_flag.en3 == 0 && encoder_flag.en4 == 0 && encoder_flag.en5 == 0){
				encoder_loop_flag=0;
			}
		}
		//////////////////////////////////////////////////////////////////////////
		//step 3
		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t2=mmToPulse(25);
		target.t3=mmToPulse(25);
		target.t4=mmToPulse(25);

		target.t5=0;

		lin_motor_Up(9);
		while(encoder_loop_flag){

			readEncoder();
			enable(9,1);

			//down

			target.t5+=encoder2.e32b_diff;

			if(target.t2_3Diff>DiffConstant){



			}

			else if(target.t2_3Diff<-DiffConstant){


			}
			else{


			}
			if(target.t2_4Diff>DiffConstant){


			}

			else if(target.t2_4Diff<-DiffConstant){


			}
			else{


			}


			if(encoder_flag.en2 == 0 && encoder_flag.en3 == 0 && encoder_flag.en4 == 0 && encoder_flag.en5 == 0){
				encoder_loop_flag=0;
			}
		}
		//////////////////////////////////////////////////////////////////////////

		break;
	case 9:

		encoder_loop_flag=1;

		encoder_reset();
		lin_reset();
		target.t1=-65000;
		target.t2=-65000;
		target.t3=-65000;
		target.t4=-65000;

		target.t5=0;

		lin_motor_Down(9);

		while(encoder_loop_flag){


			//			printf("1:%d 2:%d 3:%d 4:%d 5:%d  \r\n",encoder1.e16b_diff,encoder2.e16b_diff,encoder3.e16b_diff,encoder4.e16b_diff,encoder5.e16b_diff);
			//			printf("1.%d %d %d\r\n",encoder1.e16b_new,encoder1.e16b_diff, encoder1.e32b_new);
			//			printf("2.%d %d %d\r\n",encoder2.e16b_new,encoder2.e16b_diff, encoder2.e32b_new);
			//			printf("3.%d %d %d\r\n",encoder3.e16b_new,encoder3.e16b_diff, encoder3.e32b_new);
			//			printf("4.%d %d %d\r\n",encoder4.e16b_new,encoder4.e16b_diff, encoder4.e32b_new);
			printf("5.%d %d %d\r\n",encoder5.e16b_new,encoder5.e16b_diff, encoder5.e32b_new);

			HAL_Delay(250);

			readEncoder();
			enable(9,0);

			//down
			active.ln1=active.ln2=active.ln3=active.ln4=active.ln5=1;

			target.t5+=encoder2.e16b_diff;
			if(encoder_flag.en2 == 1){
				if(target.t2_1Diff>DiffConstant){
					active.ln2=0;

				}

				else if(target.t2_1Diff<-DiffConstant){
					active.ln1=0;

				}
				else{
					active.ln1=1;
					active.ln2=1;
				}
			}
			else{
				active.ln2=0;
				active.ln1=1;
			}


			if(target.t2_3Diff>DiffConstant){
				active.ln2=0;
			}

			else if(target.t2_3Diff<-DiffConstant){
				active.ln3=0;

			}
			else{
				active.ln2=1;
				active.ln3=1;
			}

			if(target.t2_4Diff>DiffConstant){
				active.ln2=0;
			}
			else if(target.t2_4Diff<-DiffConstant){
				active.ln4=0;
			}
			else{
				active.ln2=1;
				active.ln4=1;
			}

			set();

			//			diff16b == 0 -> stop
			lin_motor_init_en();

			if(encoder_flag.en1 == 0 && encoder_flag.en2 == 0 && encoder_flag.en3 == 0 && encoder_flag.en4 == 0 && encoder_flag.en5 == 0){
				encoder_loop_flag=0;
			}
		}


		break;
	}
}

uint16_t encoderStopWorking=25;
//upOrDown == 0 -> down 		1==Up
void enable(uint8_t sel, uint8_t upOrDown){

	if(sel == 1 || sel == 9){
		if(target.t1Diff >= 0){
			if(target.t1Diff>encoderStopWorking){
				encoder_flag.en1=1;
				if(upOrDown){
					lin_motor_Up(1);
				}
				else{
					lin_motor_Down(1);
				}
			}
			else{
				encoder_flag.en1=0;
			}
		}
		else{
			if(-target.t1Diff>encoderStopWorking){
				encoder_flag.en1=1;
				if(!upOrDown){
					lin_motor_Up(1);
				}
				else{
					lin_motor_Down(1);
				}
			}
			else{
				encoder_flag.en1=0;
			}
		}
	}
	if(sel == 2|| sel == 9){
		if(target.t2Diff >= 0){
			if(target.t2Diff>encoderStopWorking){
				encoder_flag.en2=1;
				if(upOrDown){
					lin_motor_Up(2);
				}
				else{
					lin_motor_Down(2);
				}
			}
			else{
				encoder_flag.en2=0;
			}
		}
		else{
			if(-target.t2Diff>encoderStopWorking){
				encoder_flag.en2=1;
				if(!upOrDown){
					lin_motor_Up(2);
				}
				else{
					lin_motor_Down(2);
				}
			}
			else{
				encoder_flag.en2=0;
			}
		}
	}
	if(sel == 3|| sel == 9){
		if(target.t3Diff >= 0){
			if(target.t3Diff>encoderStopWorking){
				encoder_flag.en3=1;
				if(upOrDown){
					lin_motor_Up(3);
				}
				else{
					lin_motor_Down(3);
				}
			}
			else{
				encoder_flag.en3=0;
			}
		}
		else{
			if(-target.t3Diff>encoderStopWorking){
				encoder_flag.en3=1;
				if(!upOrDown){
					lin_motor_Up(3);
				}
				else{
					lin_motor_Down(3);
				}
			}
			else{
				encoder_flag.en3=0;
			}
		}
	}

	if(sel == 4|| sel == 9){
		if(target.t4Diff >= 0){
			if(target.t4Diff>encoderStopWorking){
				encoder_flag.en4=1;
				if(upOrDown){
					lin_motor_Up(4);
				}
				else{
					lin_motor_Down(4);
				}
			}
			else{
				encoder_flag.en4=0;
			}
		}
		else{
			if(-target.t4Diff>encoderStopWorking){
				encoder_flag.en4=1;
				if(!upOrDown){
					lin_motor_Up(4);
				}
				else{
					lin_motor_Down(4);
				}
			}
			else{
				encoder_flag.en4=0;
			}
		}
	}
	if(sel == 5 || sel == 9){
		if(target.t5Diff >= 0){
			if(target.t5Diff>encoderStopWorking){
				encoder_flag.en5=1;
				if(upOrDown){
					lin_motor_Up(5);
				}
				else{
					lin_motor_Down(5);
				}
			}
			else{
				encoder_flag.en5=0;
			}
		}
		else{
			if(-target.t5Diff>encoderStopWorking){
				encoder_flag.en5=1;
				if(!upOrDown){
					lin_motor_Up(5);
				}
				else{
					lin_motor_Down(5);
				}
			}
			else{
				encoder_flag.en5=0;
			}
		}
	}
}


void set(){
	if(encoder_flag.en1 && active.ln1)	new_pwm_en(1);
	else	new_pwm_stop(1);

	if(encoder_flag.en2 && active.ln2)	new_pwm_en(2);
	else	new_pwm_stop(2);

	if(encoder_flag.en3 && active.ln3)	new_pwm_en(3);
	else	new_pwm_stop(3);

	if(encoder_flag.en4 && active.ln4)	new_pwm_en(4);
	else	new_pwm_stop(4);

	if(encoder_flag.en5 && active.ln1)	new_pwm_en(5);
	else	new_pwm_stop(5);

}





void lin_reset(){
	encoder_flag.en1 = encoder_flag.en2 = encoder_flag.en3 = encoder_flag.en4 = encoder_flag.en5 = 0;
}

void encoder_reset(){
	encoder1.e16b_old=0;
	encoder1.e32b_new=0;

	encoder2.e16b_old=0;
	encoder2.e32b_new=0;
	encoder2.e32b_old=0;

	encoder3.e16b_old=0;
	encoder3.e32b_new=0;

	encoder4.e16b_old=0;
	encoder4.e32b_new=0;

	encoder5.e16b_old=0;
	encoder5.e32b_new=0;

	__HAL_TIM_SET_COUNTER(&htim1,1);
	__HAL_TIM_SET_COUNTER(&htim2,1);
	__HAL_TIM_SET_COUNTER(&htim3,1);
	__HAL_TIM_SET_COUNTER(&htim4,1);
	__HAL_TIM_SET_COUNTER(&htim8,1);
}

//all=9, each 1~5
		void lin_motor_Down(uint8_t pos){
			switch(pos){
			case 9:
				HAL_GPIO_WritePin(DIR_linear_1_GPIO_Port, DIR_linear_1_Pin, CW);
				HAL_GPIO_WritePin(DIR_linear_2_GPIO_Port, DIR_linear_2_Pin, CW);
				HAL_GPIO_WritePin(DIR_linear_3_GPIO_Port, DIR_linear_3_Pin, CW);
				HAL_GPIO_WritePin(DIR_linear_4_GPIO_Port, DIR_linear_4_Pin, CW);
				HAL_GPIO_WritePin(DIR_linear_5_GPIO_Port, DIR_linear_5_Pin, CW);
				break;
			case 1:
				HAL_GPIO_WritePin(DIR_linear_1_GPIO_Port, DIR_linear_1_Pin, CW);
				break;
			case 2:
				HAL_GPIO_WritePin(DIR_linear_2_GPIO_Port, DIR_linear_2_Pin, CW);
				break;
			case 3:
				HAL_GPIO_WritePin(DIR_linear_3_GPIO_Port, DIR_linear_3_Pin, CW);
				break;
			case 4:
				HAL_GPIO_WritePin(DIR_linear_4_GPIO_Port, DIR_linear_4_Pin, CW);
				break;
			case 5:
				HAL_GPIO_WritePin(DIR_linear_5_GPIO_Port, DIR_linear_5_Pin, CW);
				break;
			}
		}

		//all=9, each 1~5
		void lin_motor_Up(uint8_t pos){
			switch(pos){
			case 9:
				HAL_GPIO_WritePin(DIR_linear_1_GPIO_Port, DIR_linear_1_Pin, CCW);
				HAL_GPIO_WritePin(DIR_linear_2_GPIO_Port, DIR_linear_2_Pin, CCW);
				HAL_GPIO_WritePin(DIR_linear_3_GPIO_Port, DIR_linear_3_Pin, CCW);
				HAL_GPIO_WritePin(DIR_linear_4_GPIO_Port, DIR_linear_4_Pin, CCW);
				HAL_GPIO_WritePin(DIR_linear_5_GPIO_Port, DIR_linear_5_Pin, CCW);
				break;
			case 1:
				HAL_GPIO_WritePin(DIR_linear_1_GPIO_Port, DIR_linear_1_Pin, CCW);
				break;
			case 2:
				HAL_GPIO_WritePin(DIR_linear_2_GPIO_Port, DIR_linear_2_Pin, CCW);
				break;
			case 3:
				HAL_GPIO_WritePin(DIR_linear_3_GPIO_Port, DIR_linear_3_Pin, CCW);
				break;
			case 4:
				HAL_GPIO_WritePin(DIR_linear_4_GPIO_Port, DIR_linear_4_Pin, CCW);
				break;
			case 5:
				HAL_GPIO_WritePin(DIR_linear_5_GPIO_Port, DIR_linear_5_Pin, CCW);
				break;
			}
		}

		void lin_motor_init_en(){
			if(encoder1.e16b_diff ==0){
				encoder_flag.en1=0;
			}
			if(encoder2.e16b_diff ==0){
				encoder_flag.en2=0;
			}
			if(encoder3.e16b_diff ==0){
				encoder_flag.en3=0;
			}
			if(encoder4.e16b_diff ==0){
				encoder_flag.en4=0;
			}
			if(encoder5.e16b_diff ==0){
				encoder_flag.en5=0;
			}
		}
		int32_t mmToPulse(int32_t mm){
			return mm/3.0*16.0*2.0;
		}
		//###############################################################################################

		void walker_hold(){
			HAL_GPIO_WritePin(Walker_motor_in1_GPIO_Port, Walker_motor_in1_Pin, SET);
			HAL_GPIO_WritePin(Walker_motor_in2_GPIO_Port, Walker_motor_in2_Pin, RESET);
			//wait for process complete
		}
		void walker_release(){
			HAL_GPIO_WritePin(Walker_motor_in1_GPIO_Port, Walker_motor_in1_Pin, RESET);
			HAL_GPIO_WritePin(Walker_motor_in2_GPIO_Port, Walker_motor_in2_Pin, SET);
			//wait for process complete
		}

		void stage_PUSH(){
#if !stage_disable
			if(stage_status != status_Stage_PUSH){
				if(HAL_GPIO_ReadPin(stage_PULL_SW_GPIO_Port, stage_PULL_SW_Pin)==GPIO_PIN_RESET){
					stage_PULL();
				}
				HAL_Delay(3500);
				HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
				HAL_GPIO_WritePin(lin_Wheel_Dir_GPIO_Port, lin_Wheel_Dir_Pin, CCW);
				HAL_GPIO_WritePin(lin_Wheel_En_GPIO_Port, lin_Wheel_En_Pin, RESET);
				stage_status=status_Stage_PUSH;

				stage_delay_act=1;
			}
#endif
		}

		uint8_t flag_stage;
		void stage_PULL(){
#if !stage_disable

			if(stage_status != status_Stage_PULL){
				flag_stage=1;
				HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
				HAL_GPIO_WritePin(lin_Wheel_Dir_GPIO_Port, lin_Wheel_Dir_Pin, CW);
				HAL_GPIO_WritePin(lin_Wheel_En_GPIO_Port, lin_Wheel_En_Pin, RESET);
				stage_status=status_Stage_PULL;
				stage_delay_act=1;

				while(flag_stage){
					if(HAL_GPIO_ReadPin(stage_PULL_SW_GPIO_Port, stage_PULL_SW_Pin)==GPIO_PIN_RESET){
						HAL_Delay(50);
					}
					else{
						delay_sec_stage=SET_Lin_Stage_PULL;
						flag_stage=0;
					}

					if(stage_delay_act == 0){
						flag_stage=0;
					}

				}

			}
#endif
		}


		void new_pwm_en(uint8_t sel){
			switch(sel){
			case 1:
				HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
				break;
			case 2:
				HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
				break;
			case 3:
				HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
				break;
			case 4:
				HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
				break;
			case 5:
				HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
				break;
			case 9:
				HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
				break;
			}
		}

		void new_pwm_stop(uint8_t sel){
			switch(sel){
			case 1:
				HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
				break;
			case 2:
				HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
				break;
			case 3:
				HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
				break;
			case 4:
				HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
				break;
			case 5:
				HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
				break;
			case 9:
				HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
				HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
				break;
			}
		}

		void new_pwm_set(uint8_t lin1,uint8_t lin2,uint8_t lin3,uint8_t lin4,uint8_t lin5){
			if(lin1 != 0)	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, lin1);
			else	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, idle_pwm);

			if(lin2 != 0)	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, lin2);
			else	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, idle_pwm);

			if(lin3 != 0)	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, lin3);
			else	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, idle_pwm);

			if(lin4 != 0)	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, lin4);
			else	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, idle_pwm);

			if(lin5 != 0)	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, lin5);
			else	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, idle_pwm);
		}

		uint8_t c_load = 0.7;
		void new_delay_cal(float mm,float Load, float pwm){
			float cal_time;
			if(Load == 1){
				cal_time = ((mm/10.0)*(1.0/c_load)*(1.0/(pwm/100.0)))*1000.0;
			}
			else{
				cal_time = ((mm/10.0)*(1.0/(pwm/100.0)))*1000.0;
			}
			HAL_Delay(cal_time);

		}

		void calibration_4(){

			new_pwm_set(100,100,100,100,100);

			lin_motor_Down(9);
			new_pwm_en(9);
			new_delay_cal(195, 0, 75);
			new_pwm_stop(9);

			new_pwm_set(75, 75, 73,75, 75);

			lin_motor_Up(9);
			new_pwm_en(1);
			new_delay_cal(3, 0, 75);
			new_pwm_en(2);
			new_delay_cal(12, 0, 75);
			new_pwm_en(4);
			new_delay_cal(7, 0, 75);

			new_pwm_en(3);

			new_delay_cal(0, 0, 75);

			new_pwm_en(5);
			new_delay_cal(40, 0, 75);
			new_pwm_stop(3);
			new_pwm_stop(4);
			new_pwm_stop(5);
			new_pwm_stop(2);
			new_pwm_stop(1);
		}

		void new_standing_method(uint8_t step){
			switch (step){

			case standing:
				////stand
				new_pwm_set(75, 75, 73,75, 75);
				lin_motor_Up(9);
				new_pwm_en(9);
				lin_motor_Down(1);
				new_delay_cal(62, 0, 75);
				lin_motor_Up(1);
				new_delay_cal(55, 0, 75);
				new_pwm_stop(1);
				new_delay_cal(3, 0, 75);
				new_pwm_stop(3);
				new_pwm_stop(4);
				new_delay_cal(10, 0, 75);
				new_pwm_stop(2);
				new_delay_cal(10, 0, 75);
				new_pwm_stop(5);
				break;
			case sit:
				//		////sit
				new_pwm_set(75, 75, 73,75, 75);
				lin_motor_Down(9);
				new_pwm_en(5);
				new_delay_cal(10, 0, 75);
				new_pwm_en(2);
				new_delay_cal(10, 0, 75);
				lin_motor_Up(1);
				new_pwm_en(9);
				new_delay_cal(62, 0, 75);
				lin_motor_Down(1);
				new_delay_cal(55, 0, 75);
				new_pwm_stop(1);
				new_delay_cal(3, 0, 75);
				new_pwm_stop(2);
				new_pwm_stop(3);
				new_pwm_stop(4);
				new_pwm_stop(5);
				break;
			}
		}



		void runway_Up(){
			if(runway_status != status_runway_UP){
				HAL_GPIO_WritePin(runway1_in1_GPIO_Port, runway1_in1_Pin, RESET);
				HAL_GPIO_WritePin(runway_in1_GPIO_Port, runway_in1_Pin, RESET);

				HAL_GPIO_WritePin(runway1_in2_GPIO_Port, runway1_in2_Pin, SET);
				HAL_GPIO_WritePin(runway_in2_GPIO_Port, runway_in2_Pin, SET);
				runway_status=status_runway_UP;
			}
		}
		void runway_Down(){
			if(runway_status != status_runway_Down){
				HAL_GPIO_WritePin(runway1_in1_GPIO_Port, runway1_in1_Pin, SET);
				HAL_GPIO_WritePin(runway_in1_GPIO_Port, runway_in1_Pin, SET);

				HAL_GPIO_WritePin(runway1_in2_GPIO_Port, runway1_in2_Pin, RESET);
				HAL_GPIO_WritePin(runway_in2_GPIO_Port, runway_in2_Pin, RESET);

				//Preventing slope breakage
				HAL_Delay(250);
				//
				HAL_GPIO_WritePin(runway1_in2_GPIO_Port, runway1_in2_Pin, SET);
				HAL_GPIO_WritePin(runway_in2_GPIO_Port, runway_in2_Pin, SET);
				runway_status=status_runway_Down;
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

		/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
