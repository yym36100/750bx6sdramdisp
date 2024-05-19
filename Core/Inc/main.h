/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define cam_pwdn_Pin GPIO_PIN_13
#define cam_pwdn_GPIO_Port GPIOH
#define cam_swi2c_sda_Pin GPIO_PIN_9
#define cam_swi2c_sda_GPIO_Port GPIOI
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define cam_swi2c_scl_Pin GPIO_PIN_8
#define cam_swi2c_scl_GPIO_Port GPIOI
#define touch_rst_Pin GPIO_PIN_10
#define touch_rst_GPIO_Port GPIOI
#define touch_int_Pin GPIO_PIN_11
#define touch_int_GPIO_Port GPIOI
#define touch_swi2c_sda_Pin GPIO_PIN_7
#define touch_swi2c_sda_GPIO_Port GPIOG
#define touch_swi2c_scl_Pin GPIO_PIN_3
#define touch_swi2c_scl_GPIO_Port GPIOG
#define spilcd_dc_Pin GPIO_PIN_11
#define spilcd_dc_GPIO_Port GPIOJ
#define spi_lcd_bl_Pin GPIO_PIN_6
#define spi_lcd_bl_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
