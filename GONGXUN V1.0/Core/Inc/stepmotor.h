/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stepmotor.h
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
#ifndef __STEPMOTOR_H__
#define __STEPMOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
// #include "stdio.h"
#include "usart.h"
#include "arm_math.h"
/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */

typedef struct
{

    arm_pid_instance_f32    pid;

    float                   angle_difference;

    float                   out;

} calibPIDController;

#define All 0x00

#define LF 0x01
#define RF 0x02
#define LB 0x03
#define RB 0x04

#define ARM 0x05
#define OBJ 0x06

#define TVP_Cmd 0xFD
#define FVP_Cmd 0xFB
#define EMM_Cmd 0xFD
#define SYNC_Cmd 0xFF

#define TVP_Cmd_Length 16
#define FVP_Cmd_Length 12
#define EMM_Cmd_Length 13

// negative
#define bacw            0x01
// positive
#define forw            0x00

#define relat           0x00
#define absol           0x01

#define sYnc            0x01
#define asYnc           0x00

#define targetp         0x00003E80
#define cHecksum        0x6B

#define red_Position    0x00000000
#define green_Position  0x000004B0
#define blue_Position   0x00000960

#define zero_Position 0x00000000
#define red_Arm_Position 0x000001C2
#define green_Arm_Position 0x00000000
#define blue_Arm_Position 0x000001C2
#define calib_Arm_Position 0x00000708


#define cmd_delaytime   10

/**
 *  ZDT CMD Param
 */
#define max_Velo 0x05DC
#define max_Acc 0x01F4

#define min_Velo 0x012C
/**
 *  EMM OBJ CMD Param
 */
#define obj_Velo 0x0064
#define obj_Acc 0x0F
/**
 *  EMM Rotate CMD Param
 */
#define emm_Rotate_Velo 0x007D
#define emm_Rotate_Acc 0x00
/**
 *  EMM Move CMD Param
 */
#define emm_Velo 0x007D
#define emm_Acc 0x96

    void motor_Sync_Move(void);
    void motor_Disable(void);

    /**
     *  EMM motor 
     */

    void motor_EMM_LF_Forward(uint32_t target_Position);
    void motor_EMM_RF_Forward(uint32_t target_Position);
    void motor_EMM_LB_Forward(uint32_t target_Position);
    void motor_EMM_RB_Forward(uint32_t target_Position);

    void motor_EMM_LF_Backward(uint32_t target_Position);
    void motor_EMM_RF_Backward(uint32_t target_Position);
    void motor_EMM_LB_Backward(uint32_t target_Position);
    void motor_EMM_RB_Backward(uint32_t target_Position);

    void motor_EMM_Rotate_Absol(uint32_t target_Position);
    
    /**
     *  EMM car
     */

    void car_EMM_Rotate_Clockwise(uint32_t target_Position);
    void car_EMM_Rotate_CounterClockwise(uint32_t target_Position);

    void car_Move_Forward_EMM(uint32_t target_Position);
    void car_Move_Backward_EMM(uint32_t target_Position);
    void car_Move_Leftward_EMM(uint32_t target_Position);
    void car_Move_Rightward_EMM(uint32_t target_Position);

    /**
     *  ZDT motor
     */

    void motor_FVP_LF_Forward(uint32_t target_Position);
    void motor_FVP_RF_Forward(uint32_t target_Position);
    void motor_FVP_LB_Forward(uint32_t target_Position);
    void motor_FVP_RB_Forward(uint32_t target_Position);

    void motor_FVP_LF_Backward(uint32_t target_Position);
    void motor_FVP_RF_Backward(uint32_t target_Position);
    void motor_FVP_LB_Backward(uint32_t target_Position);
    void motor_FVP_RB_Backward(uint32_t target_Position);

    void motor_TVP_LF_Forward(uint32_t target_Position);
    void motor_TVP_RF_Forward(uint32_t target_Position);
    void motor_TVP_LB_Forward(uint32_t target_Position);
    void motor_TVP_RB_Forward(uint32_t target_Position);

    void motor_TVP_LF_Backward(uint32_t target_Position);
    void motor_TVP_RF_Backward(uint32_t target_Position);
    void motor_TVP_LB_Backward(uint32_t target_Position);
    void motor_TVP_RB_Backward(uint32_t target_Position);

    /**
     *  ZDT car
     */

    void car_Move_Forward_FVP(uint32_t target_Position);
    void car_Move_Backward_FVP(uint32_t target_Position);
    void car_Move_Leftward_FVP(uint32_t target_Position);
    void car_Move_Rightward_FVP(uint32_t target_Position);

    void car_Move_Forward_TVP(uint32_t target_Position);
    void car_Move_Backward_TVP(uint32_t target_Position);
    void car_Move_Leftward_TVP(uint32_t target_Position);
    void car_Move_Rightward_TVP(uint32_t target_Position);

    void car_Rotate_Clockwise(uint32_t target_Position);
    void car_Rotate_CounterClockwise(uint32_t target_Position);

    // void wait_for_CAR_OK();
    // void wait_for_Rotate_OK();
    // void begin_Rotate();
    void clear_Motor_Position();
    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __STEPMOTOR_H__ */

