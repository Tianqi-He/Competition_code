/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    imu.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
    typedef struct IMUData_Packet_t
	{
		float gyroscope_x;          //unit: rad/s
		float gyroscope_y;          //unit: rad/s
		float gyroscope_z;          //unit: rad/s
		float accelerometer_x;      //m/s^2
		float accelerometer_y;      //m/s^2
		float accelerometer_z;      //m/s^2
		float magnetometer_x;       //mG
		float magnetometer_y;       //mG
		float magnetometer_z;       //mG
		float imu_temperature;      //C
		float Pressure;             //Pa
		float pressure_temperature; //C
		uint32_t Timestamp;          //us
} IMUData_Packet_t;

typedef struct AHRSData_Packet_t
{
	float RollSpeed;   //unit: rad/s
	float PitchSpeed;  //unit: rad/s
	float HeadingSpeed;//unit: rad/s
	float Roll;        //unit: rad
	float Pitch;       //unit: rad
	float Heading;     //unit: rad
	float Qw;//w          //Quaternion
	float Qx;//x
	float Qy;//y
	float Qz;//z
	uint32_t Timestamp; //unit: us
}AHRSData_Packet_t;

typedef struct
{
    float x;
    float y;
} _angle;

typedef struct
{

    arm_pid_instance_f32 pid;

    float angle_difference;

    float out;

} anglePIDController;
typedef struct
{
    float x;  // X坐标
    float y;  // Y坐标
    float vx; // X方向速度
    float vy; // Y方向速度
} State;

#define BUFF_SIZE	512
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN  0x38   //56+8  8组数据
#define AHRS_LEN 0x30   //48+8  7组数据
#define INSGPS_LEN 0x42 //72+8  10组数据
#define IMU_CAN 9
#define AHRS_CAN 8
#define INSGPS_CAN 11


// #define pi 3.1415926

#define pi_angle(inputangle) (inputangle < PI ? inputangle : (inputangle >= PI ? inputangle - 2 * PI : 0))

#define pi2_angle(inputangle) (inputangle < 2*PI ? inputangle: (inputangle >= 2*PI ? inputangle - 2 * PI : 0))
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */
void TTL_Hex2Dec(void);
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);

float angle_difference_byVector(float rad_target_angle, float rad_current_angle);
void angle_PID_init(anglePIDController *pid, float kp, float ki, float kd);
void angle_PID_Compute(anglePIDController *Controller, float angle_error, float dt);
void angle_to_sin_cos(float rad_angle, _angle *xy_angle);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__IMU_H__ */

