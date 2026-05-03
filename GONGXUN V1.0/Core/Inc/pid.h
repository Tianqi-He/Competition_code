/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pid.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "arm_math.h"
    /* USER CODE END Includes */

    /* USER CODE BEGIN Private defines */
    enum PID_MODE
    {
        PID_POSITION = 0,
        PID_DELTA
    };

    typedef struct
    {
        uint8_t mode;
        // PID 三参数
        float Kp;
        float Ki;
        float Kd;

        float max_out;  // 最大输出
        float max_iout; // 最大积分输出

        float set;
        float fdb;

        float out;
        float Pout;
        float Iout;
        float Dout;
        float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
        float error[3]; // 误差项 0最新 1上一次 2上上次

    } pid_type_def;

/* USER CODE END Private defines */
/**
 * 限幅
 */
#define LimitMax(input, max_abs) ((input) > (max_abs) ? (max_abs) : ((input) < -(max_abs) ? -(max_abs) : (input)))
/**
 * 死区
 */
#define LimitMin(input, min_abs) ((input) > (min_abs) ? (input) : ((input) < -(min_abs) ? (input) : -(min_abs)))



    /* USER CODE BEGIN Prototypes */
    float PID_calc(pid_type_def *pid, float ref, float set);

    void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

    void PID_clear(pid_type_def *pid);
    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */

