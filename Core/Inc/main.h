/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* LIS2MDL magnetometer register addresses (SPI) */
#define LIS2MDL_WHO_AM_I_REG            0x4FU
#define LIS2MDL_WHO_AM_I                0x40U
#define LIS2MDL_CFG_REG_A               0x60U
#define LIS2MDL_CFG_REG_B               0x61U
#define LIS2MDL_CFG_REG_C               0x62U
#define LIS2MDL_STATUS_REG              0x67U
#define LIS2MDL_OUTX_L_REG              0x68U
#define LIS2MDL_STATUS_ZYXDA            0x08U
#define LIS2MDL_CFG_REG_A_VAL           0x84U   /* COMP_TEMP_EN=1, ODR=20Hz, LP=0(HR), MD=00(continuous) */
/* CFG_REG_B = 0x03: LPF=1 (bit 0, BW=ODR/4), OFF_CANC=1 (bit 1),
 * Set_FREQ=0 (bit 2, set pulse every 64 ODR), INT_on_DataOFF=0 (bit 3),
 * OFF_CANC_ONE_SHOT=0 (bit 4, single-measurement only), bits 7:5 = 0.
 * LIS2MDL DS12144 Rev 6 section 8.6, Tables 26/27.
 * Offset cancellation is what bounds the per-part zero-gauss offset to
 * +/-60 mgauss (Table 2, TyOff) and drops RMS noise 4.5 -> 3 mgauss (Table 9).
 * Costs: 20 Hz high-resolution supply current 200 -> 235 uA (Table 10) and
 * turn-on time 9.4 ms -> 9.4 ms + 1/ODR (Table 11).
 * Set_FREQ stays 0 on purpose: Table 2 note 5 excludes magnetic-shock drift
 * from the TyOff spec, and the recurring set pulse is what recovers the offset
 * after a magnet has been near the part. Set_FREQ=1 fires it only once at
 * power-on, with no recovery short of a power cycle. */
#define LIS2MDL_CFG_REG_B_VAL           0x03U
#define LIS2MDL_CFG_REG_C_VAL           0x34U   /* I2C_DIS=1, BDU=1, 4WSPI=1 for separate SDO/MISO */
#define LIS2MDL_SENSITIVITY_MGAUSS      1.5f    /* 1.5 mgauss/LSB */
#define LIS2MDL_SENSITIVITY             0.0015f /* 1.5 mgauss/LSB in gauss */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* LIS2MDL SPI helper functions */
HAL_StatusTypeDef LIS2MDL_SPI_ReadReg(uint8_t reg, uint8_t *data, void (*cs_select)(void), void (*cs_deselect)(void));
HAL_StatusTypeDef LIS2MDL_SPI_WriteReg(uint8_t reg, uint8_t data, void (*cs_select)(void), void (*cs_deselect)(void));
HAL_StatusTypeDef LIS2MDL_SPI_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len, void (*cs_select)(void), void (*cs_deselect)(void));
HAL_StatusTypeDef LIS2MDL_CheckWhoAmI(volatile uint8_t *who_am_i, void (*cs_select)(void), void (*cs_deselect)(void));
HAL_StatusTypeDef LIS2MDL_Init(void (*cs_select)(void), void (*cs_deselect)(void));
HAL_StatusTypeDef LIS2MDL_ReadXYZ(int16_t *x, int16_t *y, int16_t *z, void (*cs_select)(void), void (*cs_deselect)(void));

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_MISO_Pin GPIO_PIN_2
#define SD_MISO_GPIO_Port GPIOC
#define SD_MOSI_Pin GPIO_PIN_1
#define SD_MOSI_GPIO_Port GPIOC
#define MAG_MOSI_Pin GPIO_PIN_5
#define MAG_MOSI_GPIO_Port GPIOB
#define MAG_MISO_Pin GPIO_PIN_4
#define MAG_MISO_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_0
#define SD_CS_GPIO_Port GPIOD
#define SD_CLK_Pin GPIO_PIN_1
#define SD_CLK_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_6
#define LED_BLUE_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOB
#define THERMISTOR2_Pin GPIO_PIN_6
#define THERMISTOR2_GPIO_Port GPIOB
#define THERMISTOR1_Pin GPIO_PIN_12
#define THERMISTOR1_GPIO_Port GPIOB
#define MAG_CS1_Pin GPIO_PIN_8
#define MAG_CS1_GPIO_Port GPIOA
#define MAG_CLK_Pin GPIO_PIN_5
#define MAG_CLK_GPIO_Port GPIOA
#define MAG_CS2_Pin GPIO_PIN_4
#define MAG_CS2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
