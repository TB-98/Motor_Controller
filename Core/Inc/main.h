/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define motor_L_DIR_Pin GPIO_PIN_2
#define motor_L_DIR_GPIO_Port GPIOE
#define motor_L_break_Pin GPIO_PIN_3
#define motor_L_break_GPIO_Port GPIOE
#define motor_R_DIR_Pin GPIO_PIN_4
#define motor_R_DIR_GPIO_Port GPIOE
#define DIR_linear_4_Pin GPIO_PIN_0
#define DIR_linear_4_GPIO_Port GPIOF
#define DIR_linear_5_Pin GPIO_PIN_1
#define DIR_linear_5_GPIO_Port GPIOF
#define motor_speed_up_Pin GPIO_PIN_2
#define motor_speed_up_GPIO_Port GPIOF
#define motor_speed_down_Pin GPIO_PIN_9
#define motor_speed_down_GPIO_Port GPIOF
#define share_seat_joy_out_Pin GPIO_PIN_0
#define share_seat_joy_out_GPIO_Port GPIOC
#define share_seat_in_Pin GPIO_PIN_3
#define share_seat_in_GPIO_Port GPIOC
#define share_standing_method_Pin GPIO_PIN_12
#define share_standing_method_GPIO_Port GPIOF
#define DIR_linear_3_Pin GPIO_PIN_0
#define DIR_linear_3_GPIO_Port GPIOG
#define share_speed_Pin GPIO_PIN_1
#define share_speed_GPIO_Port GPIOG
#define stage_PULL_SW_Pin GPIO_PIN_12
#define stage_PULL_SW_GPIO_Port GPIOE
#define share_Left_Pin GPIO_PIN_15
#define share_Left_GPIO_Port GPIOE
#define share_Right_Pin GPIO_PIN_10
#define share_Right_GPIO_Port GPIOB
#define share_Back_Pin GPIO_PIN_11
#define share_Back_GPIO_Port GPIOB
#define runway_in1_Pin GPIO_PIN_12
#define runway_in1_GPIO_Port GPIOB
#define runway_in2_Pin GPIO_PIN_13
#define runway_in2_GPIO_Port GPIOB
#define runway1_in1_Pin GPIO_PIN_14
#define runway1_in1_GPIO_Port GPIOD
#define runway1_in2_Pin GPIO_PIN_15
#define runway1_in2_GPIO_Port GPIOD
#define Walker_motor_in2_Pin GPIO_PIN_2
#define Walker_motor_in2_GPIO_Port GPIOG
#define lin_Wheel_Dir_Pin GPIO_PIN_3
#define lin_Wheel_Dir_GPIO_Port GPIOG
#define motor_R_break_Pin GPIO_PIN_8
#define motor_R_break_GPIO_Port GPIOC
#define share_default_state_Pin GPIO_PIN_9
#define share_default_state_GPIO_Port GPIOC
#define share_standing_complete_Pin GPIO_PIN_10
#define share_standing_complete_GPIO_Port GPIOC
#define share_default_set_Pin GPIO_PIN_11
#define share_default_set_GPIO_Port GPIOC
#define Walker_in_Pin GPIO_PIN_12
#define Walker_in_GPIO_Port GPIOC
#define DIR_linear_1_Pin GPIO_PIN_0
#define DIR_linear_1_GPIO_Port GPIOD
#define DIR_linear_2_Pin GPIO_PIN_1
#define DIR_linear_2_GPIO_Port GPIOD
#define Walker_motor_in1_Pin GPIO_PIN_2
#define Walker_motor_in1_GPIO_Port GPIOD
#define default_Pin GPIO_PIN_3
#define default_GPIO_Port GPIOD
#define share_speed_1_Pin GPIO_PIN_5
#define share_speed_1_GPIO_Port GPIOD
#define state_controller_Pin GPIO_PIN_6
#define state_controller_GPIO_Port GPIOD
#define lin_Wheel_En_Pin GPIO_PIN_7
#define lin_Wheel_En_GPIO_Port GPIOD
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define share_stop_out_Pin GPIO_PIN_9
#define share_stop_out_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
