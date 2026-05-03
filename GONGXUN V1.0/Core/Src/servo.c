/*******************************************************************************
* 文件名： LobotServoController.c
* 作者： 深圳乐幻索尔科技
* 日期：20160806
* LSC系列舵机控制板二次开发示例
*******************************************************************************/
#include "stm32f1xx_hal.h"
#include "servo.h"
#include <stdarg.h>
#include <string.h>

#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位

bool isUartRxCompleted = false;

uint8_t LobotTxBuf[128];  //发送缓存
uint8_t LobotRxBuf[6];
uint16_t batteryVolt;

// uint8_t Res[7];
// uint8_t Res_size = 2;
// uint8_t UART_RX_BUF[6];
// static bool isGotFrameHeader = false;

void uartWriteBuf(uint8_t *buf, uint8_t len)
{
	HAL_UART_Transmit(&huart5,buf,len,100);
}

/*********************************************************************************
 * Function:  runActionGroup
 * Description： 运行指定动作组
 * Parameters:   NumOfAction:动作组序号, Times:执行次数
 * Return:       无返回
 * Others:       Times = 0 时无限循环
 **********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
	LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;  //填充帧头
	LobotTxBuf[2] = 5;                      //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
	LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   //填充运行动作组命令
	LobotTxBuf[4] = numOfAction;            //填充要运行的动作组号
	LobotTxBuf[5] = GET_LOW_BYTE(Times);    //取得要运行次数的低八位
	LobotTxBuf[6] = GET_HIGH_BYTE(Times);   //取得要运行次数的高八位

	uartWriteBuf(LobotTxBuf, 7);            //发送
}

/*********************************************************************************
 * Function:  stopActiongGroup
 * Description： 停止动作组运行
 * Parameters:   Speed: 目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void stopActionGroup(void)
{
	LobotTxBuf[0] = FRAME_HEADER;     //填充帧头
	LobotTxBuf[1] = FRAME_HEADER;
	LobotTxBuf[2] = 2;                //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
	LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   //填充停止运行动作组命令

	uartWriteBuf(LobotTxBuf, 4);      //发送
}

/**************************舵机中断接受开启***************************************************/


// void Servo_Receiv_Start(void)
// {
// 	HAL_UART_Receive_IT(&huart5,Res ,Res_size);
// }

// /**************************舵机中断回调函数***************************************************/
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
// 	if(huart->Instance==UART5)
// 	{ //判断接收中断
// 		if(Res_size==2)
// 		{
// 			if (!isGotFrameHeader) 
// 			{  //判断帧头
// 				if ((Res[0] == FRAME_HEADER)&&(Res[1] == FRAME_HEADER)) 
// 				{
// 						isGotFrameHeader = true;
// 						Res_size=5;
// 						HAL_UART_Receive_IT(&huart5 ,Res ,Res_size);
// 				} 
// 				else 
// 				{
// 					isGotFrameHeader = false;
// 					Res_size=2;
// 					HAL_UART_Receive_IT(&huart5 ,Res ,Res_size);
// 				}
// 			}
// 		}	
// 		else if(Res_size==5)
// 		{
// 			if (isGotFrameHeader) 
// 				{ //接收接收数据部分
// 					memcpy(LobotRxBuf, Res, 5);
// 					isGotFrameHeader = false;
// 					Res_size=2;
// 					HAL_UART_Receive_IT(&huart5 ,Res ,Res_size);
// 				}
// 		}
// 		else
// 		{
// 			Res_size=2;
// 			HAL_UART_Receive_IT(&huart5 ,Res ,Res_size);
// 		}
// 	}
			
	
// }
/**************************动作组结束检查***************************************
*返回1动作组动作完成
*返回2动作组动作结束
******************************************************************************/
uint8_t action_Group_check(uint8_t numOfAction,uint8_t times)
{
	uint8_t check_flat=0;
	uint8_t check_buf[5]={0x05,0x08,numOfAction,times,0x00};
	int i;
	for(i=1;i<7;i++)
	{
		if(LobotRxBuf[i]==check_buf[i])
		{
			if(i>=4) check_flat=1;
		}
		else
		{
			check_flat=0;
			break;
		}
	}
	return check_flat;
}
