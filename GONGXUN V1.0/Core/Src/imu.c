/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    IMU.c
  * @brief   This file provides code for the configuration
  *          of the IMU instances.
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
#include "imu.h"
#include "arm_math.h"

/* USER CODE BEGIN 0 */
#include "servo.h"
extern uint32_t j,k;
IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;

static bool firstTimeAngle = true;

uint8_t Fd_rsimu[64];
uint8_t Fd_rsahrs[56];

uint8_t d;
uint8_t f;

uint8_t rs_ahrstype = 0;
uint8_t rs_imutype = 0;

extern float firYaw;

State carState;
// uint8_t correct=0;

/* USER CODE END 0 */




float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    long long transition_32;
    float tmp = 0;
    int sign = 0;
    int exponent = 0;
    float mantissa = 0;
    transition_32 = 0;
    transition_32 |= Data_4 << 24;
    transition_32 |= Data_3 << 16;
    transition_32 |= Data_2 << 8;
    transition_32 |= Data_1;
    sign = (transition_32 & 0x80000000) ? -1 : 1; // ?????ˇ???
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp = sign * mantissa * pow(2, exponent);
    return tmp;
}





void TTL_Hex2Dec(void)
{
  

        if (firstTimeAngle)
        {
            if (Fd_rsahrs[1] == TYPE_AHRS && Fd_rsahrs[2] == AHRS_LEN)
            {
                firYaw = DATA_Trans(Fd_rsahrs[27], Fd_rsahrs[28], Fd_rsahrs[29], Fd_rsahrs[30]);
                AHRSData_Packet.Heading = firYaw;
                firstTimeAngle = false;
            }
        }
        else
        {
            if (Fd_rsahrs[1] == TYPE_AHRS && Fd_rsahrs[2] == AHRS_LEN )
            {
                AHRSData_Packet.Heading = DATA_Trans(Fd_rsahrs[27], Fd_rsahrs[28], Fd_rsahrs[29], Fd_rsahrs[30]);
                firstTimeAngle = false;
            }
        }
        rs_ahrstype = 0;
        // if (Fd_rsimu[1] == TYPE_IMU && Fd_rsimu[2] == IMU_LEN)
        // {
        //     IMUData_Packet.accelerometer_x = DATA_Trans(Fd_rsimu[19], Fd_rsimu[20], Fd_rsimu[21], Fd_rsimu[22]); // ??????é?????
        //     IMUData_Packet.accelerometer_y = DATA_Trans(Fd_rsimu[23], Fd_rsimu[24], Fd_rsimu[25], Fd_rsimu[26]);
        //     rs_imutype = 0;
        //     update_state(&carState, IMUData_Packet.accelerometer_x, IMUData_Packet.accelerometer_y, 0.01);
        //     // correct++;
        // }
        // if(correct==2)
        // {
        //     correct = 0;
        //     update_state(&carState, IMUData_Packet.accelerometer_x)
        // }

               /*
    // //            IMUData_Packet.gyroscope_x = DATA_Trans(Fd_rsimu[7], Fd_rsimu[8], Fd_rsimu[9], Fd_rsimu[10]); // ?§?é?????
    // //            IMUData_Packet.gyroscope_y = DATA_Trans(Fd_rsimu[11], Fd_rsimu[12], Fd_rsimu[13], Fd_rsimu[14]);
    // //            IMUData_Packet.gyroscope_z = DATA_Trans(Fd_rsimu[15], Fd_rsimu[16], Fd_rsimu[17], Fd_rsimu[18]);


    // //            IMUData_Packet.accelerometer_z = DATA_Trans(Fd_rsimu[27], Fd_rsimu[28], Fd_rsimu[29], Fd_rsimu[30]);

    // //            IMUData_Packet.magnetometer_x = DATA_Trans(Fd_rsimu[31], Fd_rsimu[32], Fd_rsimu[33], Fd_rsimu[34]); // ???????????°???
    // //            IMUData_Packet.magnetometer_y = DATA_Trans(Fd_rsimu[35], Fd_rsimu[36], Fd_rsimu[37], Fd_rsimu[38]);
    // //            IMUData_Packet.magnetometer_z = DATA_Trans(Fd_rsimu[39], Fd_rsimu[40], Fd_rsimu[41], Fd_rsimu[42]);
    //             //
    //             //		IMUData_Packet.Timestamp=timestamp(Fd_rsimu[55],Fd_rsimu[56],Fd_rsimu[57],Fd_rsimu[58]);   //???é?????
    //             //		IMUData2PC();
    //         }
    //         rs_imutype = 0;
    //     }
    */
}

/*------------------------------------------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/
/*--------Rotate angle solution and PID-----------*/
/*------------------------------------------------*/
/*----------DSP ARM Hardware Accerate-------------*/
/*------------------------------------------------*/
/*------------------------------------------------*/

void angle_to_sin_cos(float rad_angle, _angle *xy_angle)
{
    xy_angle->y = arm_sin_f32(rad_angle);
    xy_angle->x = arm_cos_f32(rad_angle);
}

float angle_difference_byVector(float rad_target_angle, float rad_current_angle)
{
    _angle xy_target_angle, xy_current_angle;
    angle_to_sin_cos(rad_target_angle, &xy_target_angle);
    angle_to_sin_cos(rad_current_angle, &xy_current_angle);

    // 计算叉积和点积
    float cross_product = xy_target_angle.y * xy_current_angle.x -
                          xy_target_angle.x * xy_current_angle.y;
    float dot_product = xy_target_angle.x * xy_current_angle.x +
                        xy_target_angle.y * xy_current_angle.y;

    // 使用 atan2 计算两向量之间的夹角
    float angle_diff = atan2f(cross_product, dot_product);

    return angle_diff; // 返回 [-pi, pi] 范围内的角度误差
}

void angle_PID_Compute(anglePIDController *Controller, float angle_error, float dt)
{
    // 使用 PID 控制器计算控制信号
    Controller->out = arm_pid_f32(&Controller->pid, angle_error);
}

// 初始化PID控制器
void angle_PID_init(anglePIDController *pid, float kp, float ki, float kd)
{
    pid->pid.Kp = kp;
    pid->pid.Ki = ki;
    pid->pid.Kd = kd;
    arm_pid_init_f32(&pid->pid, 1); // 初始化 PID 控制器
}
 double a, b;
double errorXY,errorYaw;
double sin1,cos1,tan1;
void cali(anglePIDController *ControllerYAWXY, int32_t lastPosition,double differenceTheta)
{
    if (differenceTheta>0.001f)
    {
        // tan or sin
        //算出的基值
				sin1=arm_sin_f32(differenceTheta);
			cos1=arm_cos_f32(differenceTheta);
			tan1= sin1 /cos1;
         errorXY =  tan1  * lastPosition;
        // a = arm_pid_f32(&ControllerYAWXY->pid, errorXY);
        // 基值被加减
         errorYaw = differenceTheta;
        b = arm_pid_f32(&ControllerYAWXY->pid, errorYaw);
        motor_EMM_LF_Backward((errorXY * errorYaw) - b);
        motor_EMM_LB_Backward((errorXY * errorYaw) - b);
        motor_EMM_RF_Backward((errorXY * errorYaw) + b);
        motor_EMM_RB_Backward((errorXY * errorYaw) + b);
        motor_Sync_Move();
    }
}

