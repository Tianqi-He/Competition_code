/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stepmotor.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "stepmotor.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


/* USER CODE BEGIN 0 */
uint8_t sync_Cmd[4] =
{
    All,
    SYNC_Cmd,
    0x66,
    cHecksum,
};

uint8_t disable_Cmd[6] =
{
    All,
    0XF3,
    0xAB,
    0x00,
    sYnc,
    cHecksum,
};

uint8_t stop_Cmd[5] =
{
        All,
        0xFE,
        0x98,
        0x00,
        cHecksum,
};

uint8_t fVP_Cmd[12] =
{
    LF,
    FVP_Cmd,
    forw,
    (uint8_t)((min_Velo & 0xFF00) >> 8),
    (uint8_t)((min_Velo & 0x00FF) >> 0),
    (uint8_t)((targetp & 0xFF000000) >> 24),
    (uint8_t)((targetp & 0x00FF0000) >> 16),
    (uint8_t)((targetp & 0x0000FF00) >> 8),
    (uint8_t)((targetp & 0x000000FF) >> 0),
    relat,//absol
    sYnc,
    cHecksum,
};



uint8_t tVP_Cmd[16] =
{
    LF,
    TVP_Cmd,
    forw,
    (uint8_t)((max_Acc  & 0xFF00) >> 8),
    (uint8_t)((max_Acc  & 0x00FF) >> 0),
    (uint8_t)((max_Acc  & 0xFF00) >> 8),
    (uint8_t)((max_Acc  & 0x00FF) >> 0),
    (uint8_t)((max_Velo & 0xFF00) >> 8),
    (uint8_t)((max_Velo & 0x00FF) >> 0),
    (uint8_t)((targetp & 0xFF000000) >> 24),
    (uint8_t)((targetp & 0x00FF0000) >> 16),
    (uint8_t)((targetp & 0x0000FF00) >> 8),
    (uint8_t)((targetp & 0x000000FF) >> 0),
    relat,
    sYnc,
    cHecksum
};

uint8_t eMM_OBJ_Cmd[13] =
{
    OBJ,
    EMM_Cmd,
    forw,
    (uint8_t)((obj_Velo & 0xFF00) >> 8),
    (uint8_t)((obj_Velo & 0x00FF) >> 0),
    (uint8_t)((obj_Acc & 0xFF) >> 0),
    (uint8_t)((red_Position & 0xFF000000) >> 24),
    (uint8_t)((red_Position & 0x00FF0000) >> 16),
    (uint8_t)((red_Position & 0x0000FF00) >> 8),
    (uint8_t)((red_Position & 0x000000FF) >> 0),
    absol,
    asYnc,
    cHecksum
};

uint8_t eMM_MOTOR_Cmd[13] =
{
    LF,
    EMM_Cmd,
    forw,
    (uint8_t)((emm_Velo & 0xFF00) >> 8),
    (uint8_t)((emm_Velo & 0x00FF) >> 0),
    (uint8_t)((emm_Acc & 0xFF) >> 0),
    (uint8_t)((red_Position & 0xFF000000) >> 24),
    (uint8_t)((red_Position & 0x00FF0000) >> 16),
    (uint8_t)((red_Position & 0x0000FF00) >> 8),
    (uint8_t)((red_Position & 0x000000FF) >> 0),
    relat,
    sYnc,
    cHecksum
};

uint8_t eMM_MOTOR_Position_Cmd[3] =
{
    All,
    0x36,
    cHecksum
};

uint8_t eMM_Clear_Cmd[4] =
{
    All,
    0x0A,
    0x6D,
    cHecksum
};

uint8_t eMM_Drive_Cmd[33] =
{
        All,
        0x48,
        0xD1,
        0x01, // save
        0x19, // 1.8°
        0x02, // FOC
        0x02, // UART
        0x02, // Hold 
        0x00, // CW
        0x12, // 18细分
        0x01, // 自动插补
        0x00, // 不自动息屏
        0x03, //
        0xE8, // 开环工作电流
        0x09, //
        0x60, // 闭环堵转工作电流
        0x13, //
        0x88, // 闭环输出电压
        0x05,
        0x07,
        0x01, // 地址
        0x00, // checksum 0x6b
        0x04, // 应答received 0x04 Other 0x00 None
        0x01, // 堵转保护
        0x00, //
        0x08, // 保护转速
        0x08, //
        0x98, // 保护电流
        0x07, //
        0xD0, // 保护检测时间
        0x00, //
        0x01, // 0.1°
        cHecksum
};

// uint8_t eMM_ROTATE_Cmd[13] =
//     {
//         LF,
//         EMM_Cmd,
//         forw,
//         (uint8_t)((emm_Rotate_Velo & 0xFF00) >> 8),
//         (uint8_t)((emm_Rotate_Velo & 0x00FF) >> 0),
//         (uint8_t)((0x00 & 0xFF) >> 0),
//         (uint8_t)((red_Position & 0xFF000000) >> 24),
//         (uint8_t)((red_Position & 0x00FF0000) >> 16),
//         (uint8_t)((red_Position & 0x0000FF00) >> 8),
//         (uint8_t)((red_Position & 0x000000FF) >> 0),
//         relat,
//         sYnc,
//         cHecksum
//         };
/**
 *
 */
// void calib_PID_Compute(calibPIDController *Controller, float errorXY, float dt)
// {
//     // 使用 PID 控制器计算控制信号
//     Controller->out = arm_pid_f32(&Controller->pid, errorXY);
// }
// /**
//  *
//  */
// void calib_PID_init(calibPIDController *pid, float kp, float ki, float kd)
// {
//     pid->pid.Kp = kp;
//     pid->pid.Ki = ki;
//     pid->pid.Kd = kd;
//     arm_pid_init_f32(&pid->pid, 1); // 初始化 PID 控制器
// }

/**
 *
 */
void motor_Sync_Move(void)
{
    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;

    HAL_UART_Transmit_IT(&huart1, sync_Cmd, 4);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, sync_Cmd, 4, cmd_delaytime);
}
/**
 *
 */
void motor_Stop_Move(void)
{
    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;

    HAL_UART_Transmit_IT(&huart1, stop_Cmd, 5);

    HAL_Delay(cmd_delaytime);

    led_red_on();
    // HAL_UART_Transmit(&huart1, sync_Cmd, 4, cmd_delaytime);
}
/**
 *
 */
void motor_Disable(void)
{
    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;

    HAL_UART_Transmit_IT(&huart1, disable_Cmd, 6);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, disable_Cmd, 6, cmd_delaytime);
}
#pragma region EMM_MOTOR
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------EMM-----MOTOR-----------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/

void motor_EMM_LF_Forward(uint32_t target_Position)
{

    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = LF;

    eMM_MOTOR_Cmd[2] = forw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}
/**
 *
 */
void motor_EMM_LF_Backward(uint32_t target_Position)
{

    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = LF;

    eMM_MOTOR_Cmd[2] = bacw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}
/**
 * @param
 */
void motor_EMM_RF_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = RF;

    eMM_MOTOR_Cmd[2] = bacw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}
/**
 *
 */
void motor_EMM_RF_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = RF;

    eMM_MOTOR_Cmd[2] = forw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}
/**
 *
 */
void motor_EMM_LB_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = LB;

    eMM_MOTOR_Cmd[2] = forw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}
/**
 *
 */
void motor_EMM_LB_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = LB;

    eMM_MOTOR_Cmd[2] = bacw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}
/**
 * @brief
 *
 * @param
 *
 * @return
 */
void motor_EMM_RB_Forward(uint32_t target_Position)
{

    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = RB;

    eMM_MOTOR_Cmd[2] = bacw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}
/**
 *
 */
void motor_EMM_RB_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    eMM_MOTOR_Cmd[0] = RB;

    eMM_MOTOR_Cmd[2] = forw;

    eMM_MOTOR_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_MOTOR_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_MOTOR_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_MOTOR_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);


    HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart1, eMM_MOTOR_Cmd, EMM_Cmd_Length, cmd_delaytime);
}

#pragma endregion EMM_MOTOR

#pragma region EMM_CAR

/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*-------------------EMM----CAR------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/

/**
 *
 */
void car_EMM_Rotate_Clockwise(uint32_t target_Position)
{
    // taskENTER_CRITICAL();
    // eMM_MOTOR_Cmd[10] = relat;
    // eMM_MOTOR_Cmd[11] = sYnc;
    eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Rotate_Velo & 0xFF00) >> 8);
    eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Rotate_Velo & 0x00FF) >> 0);
    eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Rotate_Acc & 0xFF) >> 0);
    motor_EMM_LF_Forward(target_Position);
    motor_EMM_LB_Forward(target_Position);
    motor_EMM_RF_Backward(target_Position);
    motor_EMM_RB_Backward(target_Position);
    motor_Sync_Move();

    // taskEXIT_CRITICAL();
}
/**
 *
 */
void car_EMM_Rotate_CounterClockwise(uint32_t target_Position)
{
    // eMM_MOTOR_Cmd[10] = relat;
    // eMM_MOTOR_Cmd[11] = sYnc;
    eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Rotate_Velo & 0xFF00) >> 8);
    eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Rotate_Velo & 0x00FF) >> 0);
    eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Rotate_Acc & 0xFF) >> 0);
    motor_EMM_LF_Backward(target_Position);
    motor_EMM_LB_Backward(target_Position);
    motor_EMM_RF_Forward(target_Position);
    motor_EMM_RB_Forward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Backward_EMM(uint32_t target_Position)
{
    // eMM_MOTOR_Cmd[10] = relat;
    // eMM_MOTOR_Cmd[11] = sYnc;
    eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Velo & 0xFF00) >> 8);
    eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Velo & 0x00FF) >> 0);
    eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Acc & 0xFF) >> 0);
    motor_EMM_LF_Forward(target_Position);
    motor_EMM_LB_Forward(target_Position);
    motor_EMM_RF_Forward(target_Position);
    motor_EMM_RB_Forward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Forward_EMM(uint32_t target_Position)
{
    // eMM_MOTOR_Cmd[10] = relat;
    // eMM_MOTOR_Cmd[11] = sYnc;
    eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Velo & 0xFF00) >> 8);
    eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Velo & 0x00FF) >> 0);
    eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Acc & 0xFF) >> 0);
    motor_EMM_LF_Backward(target_Position);
    motor_EMM_LB_Backward(target_Position);
    motor_EMM_RF_Backward(target_Position);
    motor_EMM_RB_Backward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Leftward_EMM(uint32_t target_Position)
{
    // eMM_MOTOR_Cmd[10] = relat;
    // eMM_MOTOR_Cmd[11] = sYnc;
    eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Velo & 0xFF00) >> 8);
    eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Velo & 0x00FF) >> 0);
    eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Acc & 0xFF) >> 0);
    motor_EMM_LF_Forward(target_Position);
    motor_EMM_LB_Backward(target_Position);
    motor_EMM_RB_Forward(target_Position);
    motor_EMM_RF_Backward(target_Position);
    
    motor_Sync_Move();

}

/**
 *
 */
void car_Move_Rightward_EMM(uint32_t target_Position)
{
    // eMM_MOTOR_Cmd[10] = relat;
    // eMM_MOTOR_Cmd[11] = sYnc;
    eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Velo & 0xFF00) >> 8);
    eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Velo & 0x00FF) >> 0);
    eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Acc & 0xFF) >> 0);
    motor_EMM_LF_Backward(target_Position);
    motor_EMM_LB_Forward(target_Position);
    motor_EMM_RF_Forward(target_Position);
    motor_EMM_RB_Backward(target_Position);
    motor_Sync_Move();
}

#pragma endregion EMM_CAR

#pragma region EMM_OBJ_MOTOR

/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*--------------EMM--OBJ-motor-absolute-----------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/

/**
 * @param red_Position
 * @param green_Position
 * @param blue_Position
 */
void motor_OBJ_Rotate_Absol(uint32_t target_Position)
{
    while ((huart3.gState) != HAL_UART_STATE_READY)
        ;
    eMM_OBJ_Cmd[0] = OBJ;
    eMM_OBJ_Cmd[2] = forw;
    //    eMM_OBJ_Cmd[10] = absol;
    //    eMM_OBJ_Cmd[11] = asYnc;

    eMM_OBJ_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_OBJ_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_OBJ_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_OBJ_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart3, eMM_OBJ_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart3, eMM_OBJ_Cmd, EMM_Cmd_Length,cmd_delaytime);
}
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*--------------EMM--ARM-motor-absolute-----------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/**
 * @param zero_Position
 * @param red_Arm_Position
 * @param green_Arm_Position
 * @param calib_Arm_Position
 * @param blue_Arm_Position
 */

void motor_ARM_Rotate_Absol(uint32_t target_Position)
{
    while ((huart3.gState) != HAL_UART_STATE_READY)
        ;

    eMM_OBJ_Cmd[0] = ARM;

    // eMM_OBJ_Cmd[2] = bacw;
    //    eMM_OBJ_Cmd[11] = asYnc;

    eMM_OBJ_Cmd[6] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    eMM_OBJ_Cmd[7] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    eMM_OBJ_Cmd[8] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    eMM_OBJ_Cmd[9] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart3, eMM_OBJ_Cmd, EMM_Cmd_Length);

    HAL_Delay(cmd_delaytime);
    // HAL_UART_Transmit(&huart3, eMM_OBJ_Cmd, EMM_Cmd_Length,cmd_delaytime);
}

#pragma endregion EMM_OBJ_MOTOR

#pragma region ZDT_MOTOR

/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------ZDT--MOTOR--FVP---------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/**
 *
 */
void motor_FVP_LF_Forward(uint32_t target_Position)
{

    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;
    fVP_Cmd[0] = LF;

    fVP_Cmd[2] = forw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 *
 */
void motor_FVP_LF_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    fVP_Cmd[0] = LF;

    fVP_Cmd[2] = bacw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 * @param
 */
void motor_FVP_RF_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;
    fVP_Cmd[0] = RF;

    fVP_Cmd[2] = bacw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 *
 */
void motor_FVP_RF_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;
    fVP_Cmd[0] = RF;

    fVP_Cmd[2] = forw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 *
 */
void motor_FVP_LB_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;
    fVP_Cmd[0] = LB;

    fVP_Cmd[2] = forw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 *
 */
void motor_FVP_LB_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;
    fVP_Cmd[0] = LB;

    fVP_Cmd[2] = bacw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 * @brief
 *
 * @param
 *
 * @return
 */
void motor_FVP_RB_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;
    fVP_Cmd[0] = RB;

    fVP_Cmd[2] = bacw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 *
 */
void motor_FVP_RB_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    fVP_Cmd[0] = RB;

    fVP_Cmd[2] = forw;

    fVP_Cmd[5] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    fVP_Cmd[6] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    fVP_Cmd[7] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    fVP_Cmd[8] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, fVP_Cmd, FVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*-----------------ZDT--MOTOER--TVP---------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
void motor_TVP_LF_Forward(uint32_t target_Position)
{
    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;

    tVP_Cmd[0] = LF;

    tVP_Cmd[2] = forw;

    tVP_Cmd[9]  = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(10);
}
/**
 *
 */
void motor_TVP_LF_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    tVP_Cmd[0] = LF;

    tVP_Cmd[2] = bacw;

    tVP_Cmd[9] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(10);
}
/**
 * @param
 */
void motor_TVP_RF_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    tVP_Cmd[0] = RF;

    tVP_Cmd[2] = bacw;

    tVP_Cmd[9] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(10);
}
/**
 *
 */
void motor_TVP_RF_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    tVP_Cmd[0] = RF;

    tVP_Cmd[2] = forw;

    tVP_Cmd[9] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(10);
}
/**
 *
 */
void motor_TVP_LB_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    tVP_Cmd[0] = LB;

    tVP_Cmd[2] = forw;

    tVP_Cmd[9] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    

    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 *
 */
void motor_TVP_LB_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    tVP_Cmd[0] = LB;

    tVP_Cmd[2] = bacw;

    tVP_Cmd[9] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    

    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(cmd_delaytime);
}
/**
 * @brief
 *
 * @param
 *
 * @return
 */
void motor_TVP_RB_Forward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;

    tVP_Cmd[0] = RB;

    tVP_Cmd[2] = bacw;

    tVP_Cmd[9] = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);



    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(10);
}

/**
 *
 */
void motor_TVP_RB_Backward(uint32_t target_Position)
{
    while (huart1.gState != HAL_UART_STATE_READY)
        ;
    tVP_Cmd[0] = RB;

    tVP_Cmd[2] = forw;

    tVP_Cmd[9]  = (uint8_t)((target_Position & 0xFF000000) >> 24);
    tVP_Cmd[10] = (uint8_t)((target_Position & 0x00FF0000) >> 16);
    tVP_Cmd[11] = (uint8_t)((target_Position & 0x0000FF00) >> 8);
    tVP_Cmd[12] = (uint8_t)((target_Position & 0x000000FF) >> 0);

    
    HAL_UART_Transmit_IT(&huart1, tVP_Cmd, TVP_Cmd_Length);

    HAL_Delay(10);
}

#pragma endregion ZDT_MOTOR

#pragma region ZDT_CAR

/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*----------------ZDT----CAR----------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/**
 *
 */
void car_Rotate_Clockwise(uint32_t target_Position)
{
    // taskENTER_CRITICAL();
    fVP_Cmd[9] = absol;
    motor_FVP_LF_Forward(target_Position);
    motor_FVP_LB_Forward(target_Position);
    motor_FVP_RF_Backward(target_Position);
    motor_FVP_RB_Backward(target_Position);
    motor_Sync_Move();

    // taskEXIT_CRITICAL();
}
/**
 *
 */
void car_Rotate_CounterClockwise(uint32_t target_Position)
{
    fVP_Cmd[9] = absol;
    motor_FVP_LF_Backward(target_Position);
    motor_FVP_LB_Backward(target_Position);
    motor_FVP_RF_Forward(target_Position);
    motor_FVP_RB_Forward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Forward_FVP(uint32_t target_Position)
{
    motor_FVP_LF_Forward(target_Position);
    motor_FVP_LB_Forward(target_Position);
    motor_FVP_RF_Forward(target_Position);
    motor_FVP_RB_Forward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Backward_FVP(uint32_t target_Position)
{
    motor_FVP_LF_Backward(target_Position);
    motor_FVP_LB_Backward(target_Position);
    motor_FVP_RF_Backward(target_Position);
    motor_FVP_RB_Backward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Leftward_FVP(uint32_t target_Position)
{
    motor_FVP_LF_Forward(target_Position);
    motor_FVP_LB_Backward(target_Position);
    motor_FVP_RF_Backward(target_Position);
    motor_FVP_RB_Forward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Rightward_FVP(uint32_t target_Position)
{
    motor_FVP_LF_Backward(target_Position);
    motor_FVP_LB_Forward(target_Position);
    motor_FVP_RF_Forward(target_Position);
    motor_FVP_RB_Backward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Forward_TVP(uint32_t target_Position)
{
    motor_TVP_LF_Forward(target_Position);
    motor_TVP_LB_Forward(target_Position);
    motor_TVP_RF_Forward(target_Position);
    motor_TVP_RB_Forward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Backward_TVP(uint32_t target_Position)
{
    motor_TVP_LF_Backward(target_Position);
    motor_TVP_LB_Backward(target_Position);
    motor_TVP_RF_Backward(target_Position);
    motor_TVP_RB_Backward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Leftward_TVP(uint32_t target_Position)
{
    motor_TVP_LF_Backward(target_Position);
    motor_TVP_LB_Forward(target_Position);
    motor_TVP_RF_Forward(target_Position);
    motor_TVP_RB_Backward(target_Position);
    motor_Sync_Move();
}
/**
 *
 */
void car_Move_Rightward_TVP(uint32_t target_Position)
{
    motor_TVP_LF_Forward(target_Position);
    motor_TVP_LB_Backward(target_Position);
    motor_TVP_RF_Backward(target_Position);
    motor_TVP_RB_Forward(target_Position);
    motor_Sync_Move();
}

#pragma endregion ZDT_CAR

// void ask_Motor_Velocity()
// {
//     while ((huart1.gState) != HAL_UART_STATE_READY)
//         ;

//     HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Speed_Cmd, 3);

//     HAL_Delay(cmd_delaytime);
// }

void clear_Motor_Position()
{
    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;

    HAL_UART_Transmit_IT(&huart1, eMM_Clear_Cmd, 4);

    HAL_Delay(cmd_delaytime);
}



void motor_Responce()
{
    while ((huart1.gState) != HAL_UART_STATE_READY);

    eMM_Drive_Cmd[0] = All;

    eMM_Drive_Cmd[22] = 0x04;

    HAL_UART_Transmit_IT(&huart1, eMM_Drive_Cmd, 33);

    HAL_Delay(cmd_delaytime);
}

void motor_DisResponce()
{
    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;
    eMM_Drive_Cmd[0] = All;
    eMM_Drive_Cmd[22] = 0x00;

    HAL_UART_Transmit_IT(&huart1, eMM_Drive_Cmd, 33);

    HAL_Delay(cmd_delaytime);

    while ((huart1.gState) != HAL_UART_STATE_READY)
        ;
    eMM_Drive_Cmd[0] = 0x01;
    eMM_Drive_Cmd[22] = 0x04;

    HAL_UART_Transmit_IT(&huart1, eMM_Drive_Cmd, 33);

    HAL_Delay(cmd_delaytime);
}

void ask_Motor_Position_Error(uint8_t i)
{

        while ((huart1.gState) != HAL_UART_STATE_READY)
            ;

        eMM_MOTOR_Position_Cmd[0] = i;

        HAL_UART_Transmit_IT(&huart1, eMM_MOTOR_Position_Cmd, 3);

        HAL_Delay(cmd_delaytime);
}
