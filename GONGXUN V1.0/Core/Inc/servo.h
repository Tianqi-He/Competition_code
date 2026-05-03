/*******************************************************************************
* 文件名: LobotServoController.h
* 作者: 深圳乐幻索尔科技
* 日期：20160806
* LSC系列舵机控制板二次开发示例
*******************************************************************************/

#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f1xx_hal.h"
#include "usart.h"

typedef enum {
	false = 0, true = !false
}bool;


#define FRAME_HEADER            0x55   //帧头
#define CMD_SERVO_MOVE          0x03   //舵机移动指令
#define CMD_ACTION_GROUP_RUN    0x06   //运行动作组指令
#define CMD_ACTION_GROUP_STOP   0x07  //停止动作组指令
#define CMD_ACTION_GROUP_SPEED  0x0B  //设置动作组运行速度
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //获取电池电压指令

#define action_armStandby           0x00
#define action_armQRcode            0x01
#define action_turntableStandby     0x02
#define action_turntableTake        0x03
#define action_armadjust            0x04
#define action_armClosePut          0x05
#define action_armFarPut            0x06
#define action_armCloseTake         0x07
#define action_armFarTake           0x08


extern bool isUartRxCompleted;
extern uint8_t LobotRxBuf[6];
extern uint16_t batteryVolt;


void uartWriteBuf(uint8_t *buf, uint8_t len);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);

uint8_t action_Group_check(uint8_t numOfAction,uint8_t times);


#endif
