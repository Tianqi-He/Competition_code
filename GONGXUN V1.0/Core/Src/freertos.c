/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  *
  * 该文件实现基于 FreeRTOS 的任务创建与主任务逻辑——
  * 包含小车、机械臂、IMU、OpenMV（视觉）等子系统的任务调度与事件同步。
  *
  * 主要职责：
  * - 创建系统任务（初始化、IMU、旋转、小车、机械臂、屏幕、校准等）
  * - 定义全局事件标志（用于任务间同步）
  * - 实现小车主流程状态机（扫码区 → 取物区 → 粗加工区 → 细加工区）
  *
  * 注：文件内大量使用事件标志（Event Flags）与任务间信号来实现同步与状态机切换，
  *     注释已在关键位置说明各事件含义与流程意图，方便阅读与后续维护。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "pid.h"
#include "stepmotor.h"
#include "imu.h"
#include "led.h"
#include "servo.h"
#include "arm_math.h"
// 以上为模块依赖：
// - pid.h: 提供角度/位置 PID 控制器
// - stepmotor.h: 步进/转盘/车辆电机驱动接口
// - imu.h: IMU（姿态）数据封装与操作
// - led.h: 指示灯控制（用于心跳/故障指示）
// - servo.h: 机械臂舵机控制接口
// - arm_math.h: CMSIS DSP 库（如矩阵/滤波/数学函数）
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define origin 0
#define hard 1
#define soft 2
// 位置/状态常量说明：
// - origin: 初始/默认位置
// - hard: 粗加工区标识（hardPosition）
// - soft: 细加工区标识（softPosition）
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern AHRSData_Packet_t AHRSData_Packet;
extern uint8_t openmv_QR_Buffer[10];
extern uint8_t eMM_MOTOR_Cmd[13];
extern uint8_t eMM_OBJ_Cmd[10];
extern uint8_t eMM_Clear_Cmd[4];
extern float center_x;
extern float center_y;
extern uint8_t LF_Position[4];
extern uint8_t RF_Position[4];
extern uint8_t LB_Position[4];
extern uint8_t RB_Position[4];

float firYaw;
double maxYaw;
TaskHandle_t CAR_TASKHandle;

anglePIDController rotatePID;
anglePIDController carX;
anglePIDController carY;
// anglePIDController keepStraight;
anglePIDController KeepStraight;

int32_t lastPosition;
// calibPIDController carX;
// calibPIDController carY;

float differenceAngle = 0;
float straightError  = 0;

float caliB_X = 150.0 ;
float caliB_Y = 70.0 ;

uint32_t position,car,arm;

uint32_t nowtargetposition;

uint8_t dataGot = 0;
float targetAngle;
uint8_t car_flag=0;

uint8_t carrotate = 0;

uint8_t nowState=0;

uint8_t firColorOrder[3];
uint8_t secColorOrder[3];

bool max = true;
bool leftward;
bool beStraightCaliBrated;

uint32_t correct;

//uint32_t Init_Flag=0;
uint8_t CPU_RunInfo[512];
// 全局变量说明（简要）：
// - AHRSData_Packet: 来自 IMU 的姿态数据包（heading/yaw 等）
// - openmv_QR_Buffer: 来自 OpenMV 的二维码/颜色识别缓冲区
// - eMM_*: 与电机通信的命令缓冲区
// - center_x/center_y: 视觉检测得到的中心坐标（用于色环校准）
// - LF/RF/LB/RB_Position: 四个电机的位置反馈缓冲
// - rotatePID/carX/carY/KeepStraight: 各功能的 PID 控制器实例
// - caliB_X/caliB_Y: 目标校准图像中心坐标（经验值）
// - nowState: 当前小车所在加工阶段（0:扫码/取物/1:粗加工/2:细加工）
// - beStraightCaliBrated: 直线校准完成标志
// - CPU_RunInfo: 用于存放任务运行统计信息的输出缓存

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 33,
};
/* Definitions for IMUTASK */
osThreadId_t IMUTASKHandle;
const osThreadAttr_t IMUTASK_attributes = {
  .name = "IMUTASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) 32,
};
/* Definitions for CPUTASK */
osThreadId_t CPUTASKHandle;
const osThreadAttr_t CPUTASK_attributes = {
  .name = "CPUTASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) 16,
};
/* Definitions for CARROTATETASK */
osThreadId_t CARROTATETASKHandle;
const osThreadAttr_t CARROTATETASK_attributes = {
  .name = "CARROTATETASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 24,
};
/* Definitions for CARTASK */
osThreadId_t CARTASKHandle;
const osThreadAttr_t CARTASK_attributes = {
  .name = "CARTASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) 8,
};
/* Definitions for ARMTASK */
osThreadId_t ARMTASKHandle;
const osThreadAttr_t ARMTASK_attributes = {
    .name = "ARMTASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)24,
};
/* Definitions for SCREENTASK */
osThreadId_t SCREENTASKHandle;
const osThreadAttr_t SCREENTASK_attributes = {
  .name = "SCREENTASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 24,
};
/* Definitions for IMU_Bin_Semaphore */
osSemaphoreId_t IMU_Bin_SemaphoreHandle;
const osSemaphoreAttr_t IMU_Bin_Semaphore_attributes = {
  .name = "IMU_Bin_Semaphore"
};
/* Definitions for positionEvent */
osEventFlagsId_t positionEventHandle;
const osEventFlagsAttr_t positionEvent_attributes = {
  .name = "positionEvent"
};
/* Definitions for armEvent */
osEventFlagsId_t armEventHandle;
const osEventFlagsAttr_t armEvent_attributes = {
  .name = "armEvent"
};
/* Definitions for carEvent */
osEventFlagsId_t carEventHandle;
const osEventFlagsAttr_t carEvent_attributes = {
  .name = "carEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void CPU_TASK(void * argument);
osThreadId_t CALIBTASKHandle;
const osThreadAttr_t CALIBTASK_attributes = {
    .name = "CALIBTASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)24,
};

osThreadId_t BESTRAIGHTTASKHandle;
const osThreadAttr_t BESTRAIGHTTASK_attributes = {
    .name = "BESTRAIGHTTASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)16,
};

osThreadId_t SPEEDTASKHandle;
const osThreadAttr_t SPEEDTASK_attributes = {
    .name = "SPEEDTASK",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)16,
};
// void CAR_ROTATE_TASK(void * argument);

//  void CAR_TASK(void * argument);
void CALIB_TASK(void *argument);
void BESTRAIGHT_TASK(void *argument);
void SPEED_TASK(void *argument);
void begin_Rotate();
void wait_for_CAR_OK();
void wait_for_Rotate_OK();
void wait_for_CaliBrated_OK();
void begin_Calibration();
// void clear_Motor_Position();
// void IMU_TASK(void * argument);
/* USER CODE END FunctionPrototypes */

void Default_Task(void *argument);
void IMU_TASK(void *argument);
void CPU_TASK(void *argument);
void CAR_ROTATE_TASK(void *argument);
void CAR_TASK(void *argument);
void ARM_TASK(void *argument);
void SCREEN_TASK(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
//void configureTimerForRunTimeStats(void);
//unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
//__weak void configureTimerForRunTimeStats(void)
//{

//}

//__weak unsigned long getRunTimeCounterValue(void)
//{
//return 0;
//}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	// osSemaphoreAcquire(IMU_Bin_SemaphoreHandle,osWaitForever);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
//   /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(Default_Task, NULL, &defaultTask_attributes);

  /* creation of IMUTASK */
  IMUTASKHandle = osThreadNew(IMU_TASK, NULL, &IMUTASK_attributes);

  /* creation of CPUTASK */
//   CPUTASKHandle = osThreadNew(CPU_TASK, NULL, &CPUTASK_attributes);

  /* creation of CARROTATETASK */
  CARROTATETASKHandle = osThreadNew(CAR_ROTATE_TASK, NULL, &CARROTATETASK_attributes);

  /* creation of CARTASK */
    // CARTASKHandle = osThreadNew(CAR_TASK, NULL, &CARTASK_attributes);

  /* creation of ARMTASK */
    ARMTASKHandle = osThreadNew(ARM_TASK, NULL, &ARMTASK_attributes);

  /* creation of SCREENTASK */
    SCREENTASKHandle = osThreadNew(SCREEN_TASK, NULL, &SCREENTASK_attributes);

  /* 任务创建说明（简要）：
   * - defaultTask: 系统初始化与启动控制，优先级最高，用于协调其他任务的初始状态。
   * - IMUTASK: 读取并处理 IMU 数据（发布 yawDataGot 事件）。
   * - CAR_ROTATE_TASK: 小车角度旋转控制（由事件触发开始旋转）。
   * - CAR_TASK: 主流程状态机（扫码→取物→加工→放置），目前在部分配置下可选择创建。
   * - ARMTASK: 机械臂动作执行（响应 armEventHandle 的动作命令）。
   * - SCREENTASK: 将 OpenMV 的识别结果显示并拷贝到全局缓冲。
   */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    // BESTRAIGHTTASKHandle = osThreadNew(BESTRAIGHT_TASK, NULL, &BESTRAIGHTTASK_attributes);
    // SPEEDTASKHandle = osThreadNew(SPEED_TASK, NULL, &SPEEDTASK_attributes);
    // 创建 校准任务
    CALIBTASKHandle = osThreadNew(CALIB_TASK, NULL, &CALIBTASK_attributes);
    /* USER CODE END RTOS_THREADS */

    /* Create the event(s) */
    /* creation of positionEvent */
    positionEventHandle = osEventFlagsNew(&positionEvent_attributes);

    /* creation of armEvent */
    armEventHandle = osEventFlagsNew(&armEvent_attributes);

    /* creation of carEvent */
    carEventHandle = osEventFlagsNew(&carEvent_attributes);

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */

    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void Default_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  int a = 0;
  for(;;)
  {

		//  car_Move_Forward_EMM(0x000032A0);
        // wait_for_CAR_OK();
    if(a==0)
    {
      // 等待旋转与校准模块初始化完成后再进入主流程
      // rotateInited: CAR_ROTATE_TASK 已初始化完成
      // caliInited: CALIB_TASK 已初始化完成
      car = osEventFlagsWait(carEventHandle, rotateInited|caliInited, osFlagsWaitAll, osWaitForever);
      // 暂停旋转与校准任务，交由主流程根据需要唤醒
      vTaskSuspend(CARROTATETASKHandle);
      vTaskSuspend(CALIBTASKHandle);

      a = 1; // 仅执行一次的初始化同步
    }

	led_blue_toggle();
    
    // osDelay(100);
    osThreadTerminate(defaultTaskHandle);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_TASK */
/**
* @brief Function implementing the IMUTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_TASK */
void IMU_TASK(void *argument)
{
  /* USER CODE BEGIN IMU_TASK */
  /* Infinite loop */
  for (;;)
  {
      car = osEventFlagsWait(carEventHandle, imuRawDataReceived, osFlagsWaitAll, osWaitForever);
      // 收到原始 IMU 数据后解析、上报并通知其他任务
      TTL_Hex2Dec(); // 将接收到的十六进制原始数据转换为结构体 AHRSData_Packet
      int temp = (int)(AHRSData_Packet.Heading*1000); // 将 Heading (弧度或度，根据实现) 放大便于透传
      tjc_send_val("x0", "val", temp); // 发送到上位机/显示模块（调试/监控）
      // 通知等待 yaw/姿态数据的任务（如旋转/直行校准）
      car = osEventFlagsSet(carEventHandle, yawDataGot);
  }
  
  /* USER CODE END IMU_TASK */
}

/* USER CODE BEGIN Header_CPU_TASK */
/**
* @brief Function implementing the CPUTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CPU_TASK */
void CPU_TASK(void *argument)
{
  /* USER CODE BEGIN CPU_TASK */
//	uint8_t CPU_RunInfo[512];
//  /* Infinite loop */
  for(;;)
  {
     memset(CPU_RunInfo,0,512);
     vTaskList((char *)&CPU_RunInfo); //获取任务运行时间信息
     printf("---------------------------------------------\r\n");
     printf("任务名       任务状态     优先级     剩余栈     任务序号\r\n");
     printf("%s", CPU_RunInfo);
     printf("---------------------------------------------\r\n");
     memset(CPU_RunInfo,0,512);
     vTaskGetRunTimeStats((char *)&CPU_RunInfo);
     printf("任务名         运行计数     使用率\r\n");
     printf("%s", CPU_RunInfo);
     printf("---------------------------------------------\r\n\n");
        osDelay(1000); /* 延时500个tick */

    }
  /* USER CODE END CPU_TASK */
}

/* USER CODE BEGIN Header_CAR_ROTATE_TASK */
/**
* @brief Function implementing the CARROTATETASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAR_ROTATE_TASK */
void CAR_ROTATE_TASK(void *argument)
{
  /* USER CODE BEGIN CAR_ROTATE_TASK */
      /* Infinite loop */
  float exceptedRotateAngle, initialAngle, currentAngle,dt;
  float static targetAngle;
  angle_PID_init(&rotatePID, 0.57 * 1000, 0, 1.1* 10);
  exceptedRotateAngle = PI / 2.0f;
    // exceptedRotateAngle = PI / 4.0f;
  car = osEventFlagsWait(carEventHandle, yawDataGot, osFlagsWaitAll, osWaitForever);
  initialAngle = AHRSData_Packet.Heading;
  targetAngle = initialAngle + exceptedRotateAngle;
  car = osEventFlagsSet(carEventHandle, rotateInited);
  /* Infinite loop */
  for (;;)
  {
      car = osEventFlagsWait(carEventHandle, rotateGO, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
      {    
        // 禁用 UART IDLE 中断，避免在旋转控制期间产生干扰性回调
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
        // 等待最新的 yaw 数据用于 PID 计算
        car = osEventFlagsWait(carEventHandle, yawDataGot, osFlagsWaitAll, osWaitForever);
        currentAngle = AHRSData_Packet.Heading;
        // 计算目标角度与当前角度之间的差值（考虑角度环绕）
        differenceAngle = angle_difference_byVector(targetAngle, currentAngle);
        // 使用角度差作为 PID 输入来产生输出控制量
        angle_PID_Compute(&rotatePID, differenceAngle, dt);
        // 根据 PID 输出选择顺时针或逆时针旋转命令驱动电机
        if (rotatePID.out > 0)
        {
          car_EMM_Rotate_Clockwise(rotatePID.out);
        }
        else
        {
          car_EMM_Rotate_CounterClockwise(-rotatePID.out);
        }
        // 当角度误差足够小（收敛）时，停止并准备下一次旋转
        if (fabs(differenceAngle) < 0.002f)
        {
          motor_Stop_Move();
          // 重置 PID 并设置下一次旋转的目标（通常为 +90°）
          angle_PID_init(&rotatePID, 0.57 * 1000, 0, 1.1* 10);
          exceptedRotateAngle = PI / 2.0f;
          initialAngle = AHRSData_Packet.Heading;
          targetAngle = initialAngle + exceptedRotateAngle;
          // 通知系统旋转初始化完成
          car = osEventFlagsSet(carEventHandle, rotateInited);

          // 清除准备/进行标志，置位完成标志
          car = osEventFlagsClear(carEventHandle, carReady);
          car = osEventFlagsClear(carEventHandle, rotateGO);
          car = osEventFlagsSet(carEventHandle, rotateAfter);
        }
      }    
  }


  /* USER CODE END CAR_ROTATE_TASK */
}

/* USER CODE BEGIN Header_CAR_TASK */
/**
* @brief Function implementing the CARTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAR_TASK */
void CAR_TASK(void *argument)
{
  /* USER CODE BEGIN CAR_TASK */
  /* USER CODE BEGIN CAR_task */
//  osEventFlagsSet(positionEventHandle, softPosition);
//  osEventFlagsSet(positionEventHandle, hardPosition);
      osEventFlagsSet(positionEventHandle, qRPosition);

  //  osEventFlagsSet(positionEventHandle, oBJPosition);
  //   firColorOrder[0] = '1';
  //   firColorOrder[1] = '2';
  //   firColorOrder[2] = '3';

  //   osEventFlagsWait(positionEventHandle, qRPosition | oBJPosition, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
  /* CAR_TASK 主循环：实现位置状态机（基于 positionEventHandle 事件标志）
   * 状态包括：
   * - qRPosition: 扫码区（识别二维码以得到物块处理顺序）
   * - oBJPosition: 取物区（按二维码顺序抓取物块）
   * - hardPosition: 粗加工区（搬运并旋转、定位）
   * - softPosition: 细加工区（视觉+校准+放置）
   *
   * 每个状态通过设置/清除 positionEventHandle 标志来切换流程。
   */
  while (1)
  {
//		osEventFlagsWait(positionEventHandle, qRPosition | oBJPosition, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
      // Get the flags
      position = osEventFlagsGet(positionEventHandle);

      // Check the flags
      switch (position)
      {
        /*
        扫码区
        */
      case qRPosition:
          nowState = 0;
          car = osEventFlagsSet(carEventHandle, keepStraight);
          nowtargetposition = 0x000007DF;
          car_Move_Forward_EMM(0x000007DF);
          wait_for_CAR_OK();
          nowtargetposition = 0x00002399;
          leftward = true;
          car_Move_Rightward_EMM(0x00002399);
          wait_for_CAR_OK();
        //   /*

          // 请求机械臂执行扫码就位动作（armQRStart），等待机械臂动作完成
          arm = osEventFlagsSet(armEventHandle, armQRStart);
          // 等机械臂就位完成（action 完成会置 armFinish）
          car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
          // 打开 OpenMV 扫码模式（mode=1），等待 taskCodeGot 事件（扫码结果）
          openMV_Work_Mode(1);//扫码
          car = osEventFlagsWait(carEventHandle, taskCodeGot, osFlagsWaitAll, osWaitForever);
          // 读取到二维码后关闭摄像头并让机械臂回到待机位
          openMV_Work_Mode(0); //关
          arm = osEventFlagsSet(armEventHandle, armStandby);
          car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);

        //   */
          // 左移到转盘区域
          // 到位后清楚切换标志
          position = osEventFlagsClear(positionEventHandle, qRPosition);
          position = osEventFlagsSet(positionEventHandle, oBJPosition);
          break; // Add break to avoid falling through to the next case
                 /*
                 取物区
                 */
      case oBJPosition:
		// car=osEventFlagsWait(carEventHandle, carReady, osFlagsWaitAll, osWaitForever);
        nowtargetposition = 0x00003500;
        car_Move_Rightward_EMM(0x00003500);
        wait_for_CAR_OK();
        // car_Move_Leftward_EMM(0x000001800);
        // car = osEventFlagsWait(carEventHandle, carGo, osFlagsWaitAll, osWaitForever);
        // car = osEventFlagsWait(carEventHandle, carReady, osFlagsWaitAll, osWaitForever);
        // lastPosition = 13824;
        // car = osEventFlagsSet(carEventHandle, keepStraight);
        // osThreadResume(BESTRAIGHTTASKHandle);
        // car = osEventFlagsWait(carEventHandle, calibYawAfter, osFlagsWaitAll, osWaitForever);
        // osThreadSuspend(BESTRAIGHTTASKHandle);
        // __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        //做 物块等待抓动作
        /**/
        nowtargetposition = 0x000001DF;
        car_Move_Rightward_EMM(0x000001DF);
        wait_for_CAR_OK();

        // /*
        // 按照二维码获得的顺序抓取 3 个物块（firColorOrder）
        // 每次循环：
        // 1) 旋转料盘到目标颜色位置
        // 2) 等待料盘就位（oBJReady）
        // 3) 机械臂就位并用 OpenMV 确认颜色
        // 4) 执行抓取动作（turntableTake）并等待完成
        for (int i = 0; i <= 2; i++)
        {
            switch (firColorOrder[i])
            {
                case '1':
                    //料盘 转至红色
                    motor_OBJ_Rotate_Absol(red_Position);
                    //osEventFlagsWait(armEventHandle, oBJGo, osFlagsWaitAll, osWaitForever);
                    car = osEventFlagsWait(carEventHandle, oBJReady, osFlagsWaitAll, osWaitForever);
                    // 做 抓物体等待动作
                    arm = osEventFlagsSet(armEventHandle, turntableStandby);
                    // vTaskResume(armEventHandle);
                    // 等 机械臂到位
                    car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    // vTaskSuspend(armEventHandle);
                    // openmv颜色识别
                    openMV_Work_Mode(2);
                    // 红颜色
                    car = osEventFlagsWait(carEventHandle, redDetected, osFlagsWaitAll, osWaitForever);
                    // 做 物块等待抓动作
                    arm = osEventFlagsSet(armEventHandle, turntableTake);
                    // vTaskResume(armEventHandle);
                    // 关闭
                    openMV_Work_Mode(0);
                    // 等 机械臂到位
                    car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    // vTaskSuspend(armEventHandle);
                    break;
                case '2':
                    // 料盘 转至绿色
                    motor_OBJ_Rotate_Absol(green_Position);
                    //osEventFlagsWait(armEventHandle, oBJGo, osFlagsWaitAll, osWaitForever);
                    car = osEventFlagsWait(carEventHandle, oBJReady, osFlagsWaitAll, osWaitForever);
                    // 做 抓物体等待动作
                    arm = osEventFlagsSet(armEventHandle, turntableStandby);

                    // 等 机械臂到位
                    car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    // openmv颜色识别
                    openMV_Work_Mode(2);
                    // 绿颜色
                    car = osEventFlagsWait(carEventHandle, greenDetected, osFlagsWaitAll, osWaitForever);
                    // 做 物块等待抓动作
                    arm = osEventFlagsSet(armEventHandle, turntableTake);
                    // 关闭
                    openMV_Work_Mode(0); 
                    // 等 机械臂到位
                    car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    break;
                case '3':
                    // 料盘 转至蓝色
                    motor_OBJ_Rotate_Absol(blue_Position);
                    //osEventFlagsWait(armEventHandle, oBJGo, osFlagsWaitAll, osWaitForever);
                    car = osEventFlagsWait(carEventHandle, oBJReady, osFlagsWaitAll, osWaitForever);
                    // 做 抓物体等待动作
                    arm = osEventFlagsSet(armEventHandle, turntableStandby);
                    // 等 机械臂到位
                    car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    // openmv颜色识别
                    openMV_Work_Mode(2);
                    // 蓝颜色
                    car = osEventFlagsWait(carEventHandle, blueDetected, osFlagsWaitAll, osWaitForever);
                    // 做 物块等待抓动作
                    arm = osEventFlagsSet(armEventHandle, turntableTake);
                    // 关闭
                    openMV_Work_Mode(0); 
                    // 等 机械臂到位
                    car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    break;
            }
        }
        // */

        position = osEventFlagsClear(positionEventHandle, oBJPosition);
	    position = osEventFlagsSet(positionEventHandle, hardPosition);
        break; // Add break to avoid falling through to the next case
        /*
       粗加工区
       */
      case hardPosition:
          nowtargetposition = 0x00001A00;
        // nowtargetposition = 0x000000FF;
        // 进入粗加工区：先移动到工位，然后做旋转-移动-旋转的序列以完成粗加工定位
        leftward = false;
        car_Move_Leftward_EMM(nowtargetposition);
        wait_for_CAR_OK();
        // 请求一次旋转动作并等待完成（由 CAR_ROTATE_TASK 执行）
        begin_Rotate();
        wait_for_Rotate_OK();
        nowState = 1; // 更新状态，供直行校准等子模块使用
        // 继续向左移动到下一个位置（可能为多段移动以确保定位）
        nowtargetposition = 0x000033FF;
        car_Move_Leftward_EMM(nowtargetposition);
        wait_for_CAR_OK();
        nowtargetposition = 0x000033FF;
        car_Move_Leftward_EMM(nowtargetposition);
        wait_for_CAR_OK();
        // 再次执行旋转以完成粗加工区的姿态调整
        begin_Rotate();
        wait_for_Rotate_OK();
        nowState = 2; // 准备进入细加工区
        // 切换 positionEvent 到细加工区
        position = osEventFlagsClear(positionEventHandle, hardPosition);
        position = osEventFlagsSet(positionEventHandle, softPosition);
          break; // Add break to avoid falling through to the next case
          /*
          细加工区
          */
    case softPosition:
        // nowtargetposition = 0x0000079C;
        // leftward = true;
        // car_Move_Rightward_EMM(0x0000079C);
        // wait_for_CAR_OK();
        eMM_OBJ_Cmd[2] = bacw;
        // 做 旋转机械臂yaw 至前方calib_Arm_Position
        motor_ARM_Rotate_Absol(calib_Arm_Position);
        // 做 看圆环动作
        arm = osEventFlagsSet(armEventHandle, armAdjust);
        // 等 机械臂yaw运动到位
        car = osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
        // 等 机械臂 看圆环动作 运动到位
        car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
        // 开 摄像头色环校准
        // 启动视觉色环校准（OpenMV mode=3），并唤醒 CALIB_TASK 执行 PID 精准校准
        openMV_Work_Mode(3);
        begin_Calibration(); // 恢复并触发 CALIB_TASK 中的 calibGo 事件
        // 等待 CALIB_TASK 设置 caliBrated 标志（表明 X/Y 校准已完成）
        wait_for_CaliBrated_OK();
        // car = osEventFlagsWait(carEventHandle, caliBrated, osFlagsWaitAll, osWaitForever);
        // osThreadSuspend(CALIBTASKHandle);
		// __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        // 关 摄像头色环校准
        openMV_Work_Mode(0);
        // nowtargetposition = 0x0000077C;
        // leftward = false;
        // car_Move_Leftward_EMM(0x0000077C);
        // wait_for_CAR_OK();
        // 放下粗加工的物料 按 firColorOrder 顺序
        for (int i = 0; i <= 2; i++)
        {
            switch (firColorOrder[i])
            {
            case '1':
                // 料盘 转至红色
                motor_OBJ_Rotate_Absol(red_Position);
                // 做 旋转机械臂 armStandby
                arm = osEventFlagsSet(armEventHandle, armStandby);
                // 等 料盘 运动到位
                car = osEventFlagsWait(carEventHandle, oBJReady, osFlagsWaitAll, osWaitForever);
                // 等 机械臂 看圆环动作 运动到位
                car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                // 做 旋转机械臂yaw 至前方zero_Position 拿物料
                motor_ARM_Rotate_Absol(zero_Position);
                // 等 机械臂yaw运动到位
                car = osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                // 做 放红色物块 动作
                arm = osEventFlagsSet(armEventHandle, armFarPut);
                eMM_OBJ_Cmd[2] = forw;
                // 做 旋转机械臂yaw 至前方red_Arm_Position
                motor_ARM_Rotate_Absol(red_Arm_Position);
                // 等 机械臂yaw运动到位
                car = osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                // 等 机械臂到位
                car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                break;
            case '2':
                motor_OBJ_Rotate_Absol(green_Position);
                // 等 料盘 运动到位
                car = osEventFlagsWait(carEventHandle, oBJReady, osFlagsWaitAll, osWaitForever);
                // 做 旋转机械臂 armStandby
                arm = osEventFlagsSet(armEventHandle, armStandby);
                // 等 机械臂 看圆环动作 运动到位
                car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                // 做 旋转机械臂yaw 至前方zero_Position 拿物料
                motor_ARM_Rotate_Absol(zero_Position);
                // 等 机械臂yaw运动到位
                car = osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                // 做 放绿色物块 动作
                osEventFlagsSet(armEventHandle, armClosePut);
                eMM_OBJ_Cmd[2] = bacw;
                // 料盘 转至绿色
                motor_ARM_Rotate_Absol(green_Arm_Position);
                // 等 机械臂yaw运动到位
                car = osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                // 等 机械臂到位
                car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                break;
            case '3':
                motor_OBJ_Rotate_Absol(blue_Position);
                // 等 料盘 运动到位
                car = osEventFlagsWait(carEventHandle, oBJReady, osFlagsWaitAll, osWaitForever);
                // 做 旋转机械臂 armStandby
                arm = osEventFlagsSet(armEventHandle, armStandby);
                // 等 机械臂 看圆环动作 运动到位
                car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                // 做 旋转机械臂yaw 至前方zero_Position 拿物料
                motor_ARM_Rotate_Absol(zero_Position);
                // 等 机械臂yaw运动到位
                car = osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                arm = osEventFlagsSet(armEventHandle, armFarPut);
                eMM_OBJ_Cmd[2] = bacw;
                // 料盘 转至蓝色
                motor_ARM_Rotate_Absol(blue_Arm_Position);
                // 做 放蓝色物块 动作
                arm = osEventFlagsSet(armEventHandle, armFarPut);
                // 等 机械臂yaw运动到位
                car = osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                // 等 机械臂到位
                car = osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                break;
            }
            }
            /*
            // 拾起粗加工的物料 按 firColorOrder 顺序
            for (int i = 0; i <= 2; i++)
            {
                switch (firColorOrder[i])
                {
                case '1':
                    // 做 旋转机械臂yaw 至前方red_Arm_Position
                    motor_ARM_Rotate_Absol(red_Arm_Position);
                    // 等 机械臂yaw运动到位
                    osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                    // 做 抓物体等待动作
                    osEventFlagsSet(armEventHandle, turntableStandby);
                    // 等 机械臂到位
                    osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    // 红颜色
                    osEventFlagsWait(carEventHandle, redDetected, osFlagsWaitAll, osWaitForever);
                    // 做 物块等待抓动作
                    osEventFlagsSet(armEventHandle, turntableTake);
                    // 等 机械臂到位
                    osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    break;
                case '2':
                    // 料盘 转至绿色
                    motor_ARM_Rotate_Absol(green_Arm_Position);
                    // 等 机械臂yaw运动到位
                    osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                    // 做 抓物体等待动作
                    osEventFlagsSet(armEventHandle, turntableStandby);
                    // 等 机械臂到位
                    osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    // openmv颜色识别
                    openMV_Work_Mode(2);
                    // 绿颜色
                    osEventFlagsWait(carEventHandle, greenDetected, osFlagsWaitAll, osWaitForever);
                    // 做 物块等待抓动作
                    osEventFlagsSet(armEventHandle, turntableTake);
                    // 关闭
                    openMV_Work_Mode(0);
                    // 等 机械臂到位
                    osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    break;
                case '3':
                    // 料盘 转至蓝色
                    motor_ARM_Rotate_Absol(blue_Arm_Position);
                    // 等 机械臂yaw运动到位
                    osEventFlagsWait(carEventHandle, aRMReady, osFlagsWaitAll, osWaitForever);
                    // 做 抓物体等待动作
                    osEventFlagsSet(armEventHandle, turntableStandby);
                    // 等 机械臂到位
                    osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    // openmv颜色识别
                    openMV_Work_Mode(2);
                    // 蓝颜色
                    osEventFlagsWait(carEventHandle, blueDetected, osFlagsWaitAll, osWaitForever);
                    // 做 物块等待抓动作
                    osEventFlagsSet(armEventHandle, turntableTake);
                    // 关闭
                    openMV_Work_Mode(0);
                    // 等 机械臂到位
                    osEventFlagsWait(carEventHandle, armFinish, osFlagsWaitAll, osWaitForever);
                    break;
                }
            }
            */
            position = osEventFlagsClear(positionEventHandle, softPosition);
            break; // Add break to avoid falling through to the next case
        case DoftPosition:
            nowtargetposition = 0x000032A0;
            car_Move_Forward_EMM(0x000032A0);
            car = osEventFlagsWait(carEventHandle, carGo, osFlagsWaitAll, osWaitForever);
            car = osEventFlagsWait(carEventHandle, carReady, osFlagsWaitAll, osWaitForever);
            position = osEventFlagsClear(positionEventHandle, DoftPosition);
            nowtargetposition = 0x000032A0;
            car_Move_Rightward_EMM(0x000032A0);
            car = osEventFlagsWait(carEventHandle, carGo, osFlagsWaitAll, osWaitForever);
            car = osEventFlagsWait(carEventHandle, carReady, osFlagsWaitAll, osWaitForever);
            break;
        default : 
            while (1)
            {

                osDelay(100);
            }
            break;
      }
      // Wait for any of the specified flags to be set
      
  } 
  /* USER CODE END CAR_TASK */
}

/* USER CODE BEGIN Header_ARM_TASK */
/**
* @brief Function implementing the ARMTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ARM_TASK */
void ARM_TASK(void *argument)
{
  /* USER CODE BEGIN ARM_TASK */
  /* Infinite loop */
  for(;;)
  {
    uint32_t arm;

    // ARMTASK: 根据 armEventHandle 的标志执行预定义的动作组
    // 动作组 (action_*) 在其他文件中定义，runActionGroup 调用会执行一系列舵机/电机动作并在完成时置位 armFinish
    // 常见动作：armStandby、armQRStart、turntableStandby、turntableTake、armClosePut/armFarPut 等
		
		arm = osEventFlagsWait
		(
		armEventHandle
		,
		armStandby|
		armQRStart|
		turntableStandby|
		turntableTake|
		armClosePut|
		armFarPut|
		armCloseTake|
		armFarTake|
		armAdjust
		,
		osFlagsNoClear|osFlagsWaitAny
		,
		osWaitForever);
		
    arm = osEventFlagsGet(armEventHandle);

    switch (arm)
    {
        /*
        待机动作
        */
        case armStandby:
            // osEventFlagsWait(armEventHandle, armStandby, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_armStandby, 1);
            arm = osEventFlagsClear(armEventHandle,armStandby);
            break;
        /*
        扫码动作
        */
        case armQRStart:
            // osEventFlagsWait(armEventHandle, armQRStart, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_armQRcode, 1);
            arm = osEventFlagsClear(armEventHandle, armQRStart);
            break;
        /*
        转盘待抓动作
        */
        case turntableStandby:
            // osEventFlagsWait(armEventHandle, turntableStandby, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_turntableStandby, 1);
            arm = osEventFlagsClear(armEventHandle, turntableStandby);
            break;
        /*
        转盘抓动作
        */
        case turntableTake:
            // osEventFlagsWait(armEventHandle, turntableTake, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_turntableTake, 1);
            arm = osEventFlagsClear(armEventHandle, turntableTake);
            break;
        /*
        放 近动作
        */
        case armClosePut:
            // osEventFlagsWait(armEventHandle, armClosePut, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_armClosePut, 1);
            arm = osEventFlagsClear(armEventHandle, armClosePut);
            break;
        /*
        放 远动作
        */
        case armFarPut:
            // osEventFlagsWait(armEventHandle, armFarPut, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_armFarPut, 1);
            arm = osEventFlagsClear(armEventHandle, armFarPut);
            break;
        /*
        拿 近动作
        */
        case armCloseTake:
            // osEventFlagsWait(armEventHandle, armCloseTake, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_armCloseTake, 1);
            arm = osEventFlagsClear(armEventHandle, armCloseTake);
            break;
        /*
        拿 远动作
        */
        case armFarTake:
            // osEventFlagsWait(armEventHandle, armFarTake, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_armFarTake, 1);
            arm = osEventFlagsClear(armEventHandle, armFarTake);
            break;
        /*
        校准动作
        */
        case armAdjust:
            // osEventFlagsWait(armEventHandle, armAdjust, osFlagsWaitAll, osWaitForever);
            runActionGroup(action_armadjust, 1);
            arm = osEventFlagsClear(armEventHandle, armAdjust);
            break;
        default:
            //osDelay(100);
            break;
    }
     
  }
  /* USER CODE END ARM_TASK */
}

/* USER CODE BEGIN Header_SCREEN_TASK */
/**
* @brief Function implementing the SCREENTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SCREEN_TASK */
void SCREEN_TASK(void *argument)
{
  /* USER CODE BEGIN SCREEN_TASK */
  /* Infinite loop */
  for(;;)
  {
      // 等待二维码解析完成（taskCodeGot），并把结果展示到屏幕/上位机，同时复制到处理序列数组
      osEventFlagsWait(carEventHandle, taskCodeGot, osFlagsWaitAll, osWaitForever);
      tjc_send_txt("t0", "txt", openmv_QR_Buffer); // 将 QR 解析文本显示到屏幕
      osDelay(10);
      // 拷贝首组与次组颜色顺序：firColorOrder 存前三位，secColorOrder 存后三位
      memcpy(firColorOrder, openmv_QR_Buffer, 3);
      memcpy(secColorOrder, &openmv_QR_Buffer[4], 3);
      while ((huart4.gState) != HAL_UART_STATE_READY)
        ;
      // 任务完成后自终止，节约资源（如果需要可改为循环常驻）
      osThreadTerminate(SCREENTASKHandle);
  }
  /* USER CODE END SCREEN_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE BEGIN Header_CALIB_TASK */
/**
 * @brief Function implementing the CALIB_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CALIB_TASK */
void CALIB_TASK(void *argument)
{
    /* USER CODE BEGIN CALIB_TASK */
  // CALIB_TASK：负责视觉色环的精密 PID 校准（X/Y），用于细加工区放置精度
  // carX/carY 为用于图像偏差的 PID 控制器，输出驱动小车微小移动实现校准
  angle_PID_init(&carX, 0.7, 0, 0.03);
  angle_PID_init(&carY, 1.0, 0, 0.03);
    static bool isXCalibrated = false;
    osEventFlagsSet(carEventHandle, caliInited);
    /* Infinite loop */
    for (;;)
    {
        car = osEventFlagsWait(carEventHandle, calibGo, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
        {
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
            osEventFlagsWait(carEventHandle, xyGot, osFlagsWaitAll, osWaitForever);
            // 校准X轴
            float errorX = center_x - caliB_X;
            if (isXCalibrated == false)
            {
                while ((errorX > 1) | (errorX < -1)) // 修改条件以确保误差大于1时才进行校准
                {
                    float dt = 0;
                    angle_PID_Compute(&carX, errorX, dt);
                    if (carX.out > 0)
                    {
                        car_Move_Leftward_EMM(carX.out);
                    }
                    else
                    {

                        car_Move_Rightward_EMM(-carX.out);
                    }
                    // 检查是否完成X轴校准
                    errorX = center_x - caliB_X;
                    if (fabs(errorX) <= 0.5)
                    {
                        isXCalibrated = true; // 标记X轴已校准
                        break;                // 退出X轴校准循环
                    }
                }
            }
            // 如果X轴已校准，则校准Y轴
            else
            {
                float errorY = center_y - caliB_Y;
                while ((errorY > 1) | (errorY < -1)) // 修改条件以确保误差大于1时才进行校准
                {
                    float dt;
                    angle_PID_Compute(&carY, errorY, dt);
                    // 假设您有类似的carY处理
                    // 类似地处理Y轴的移动
                    if (carY.out > 0)
                    {
                        car_Move_Forward_EMM(carY.out);
                    }
                    else
                    {
                        car_Move_Backward_EMM(-carY.out);
                    }
                    // 更新errorY以检查是否完成校准
                    errorY = center_y - caliB_Y;
                    if (fabs(errorY) <= 0.5)
                    {
                        isXCalibrated = false;
                        motor_Stop_Move();
                        
                        // angle_PID_init(&carX, 0.7, 0, 0.03);
                        // angle_PID_init(&carY, 1.0, 0, 0.01);
                        
                        osEventFlagsClear(carEventHandle, carGo);
                        osEventFlagsClear(carEventHandle, carReady);
                        osEventFlagsClear(carEventHandle, calibGo);
                        osEventFlagsSet(carEventHandle, caliBrated);
                        break; // 退出Y轴校准循环
                    }
                }
            }
        }
    }
    /* USER CODE END CALIB_TASK */
}

/* USER CODE BEGIN Header_BESTRAIGHT_TASK_TASK */
/**
 * @brief Function implementing the BESTRAIGHT_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Heade_BESTRAIGHT_TASK_TASK */
void BESTRAIGHT_TASK(void *argument)
{
    /* USER CODE BEGIN BESTRAIGHT_TASK */
  // BESTRAIGHT_TASK：当需要保持或校准直线行驶时被触发
  // 使用 KeepStraight PID 根据 IMU YAW 修正车体姿态，保证直线运动
  double  currentAngle,dt=0;
    double static targetAngle;
    angle_PID_init(&KeepStraight, 0.57 * 1000, 0, 1.1 * 10);
    osEventFlagsWait(carEventHandle,yawDataGot, osFlagsWaitAll, osWaitForever);
    // targetAngle = firYaw;
    // static uint32_t count;
    /* Infinite loop */
    for (;;)
    {
        car = osEventFlagsWait(carEventHandle, keepStraight, osFlagsWaitAll|osFlagsNoClear, osWaitForever);
        {
            switch (nowState)
            {
                case 0:
                    targetAngle = firYaw;
                    break;
                case 1:
                    targetAngle = targetAngle + PI / 2;
                    break;
                case 2:
                    targetAngle = targetAngle + PI / 2;
                    break;
            }
            // 
            osEventFlagsWait(carEventHandle, yawDataGot, osFlagsWaitAll, osWaitForever);
            currentAngle = AHRSData_Packet.Heading;
            // 顺负逆正
            straightError = angle_difference_byVector(targetAngle, currentAngle);
            if (fabs(straightError) > 0.02f)//误差超过多少开始校准
            {
                // 
                // motor_Responce();
                
                // motor_DisResponce();
                while (fabs(straightError) > 0.02f)
                {
                    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
                    osEventFlagsWait(carEventHandle, yawDataGot, osFlagsWaitAll, osWaitForever);
                    currentAngle = AHRSData_Packet.Heading;
                    straightError = angle_difference_byVector(targetAngle, currentAngle);
                    angle_PID_Compute(&KeepStraight, straightError, dt);
                    motor_Stop_Move();
                    ask_Motor_Position_Error(0x01);
                    osEventFlagsWait(carEventHandle, positionGot, osFlagsWaitAll, osWaitForever);
                    if (KeepStraight.out > 0)
                    {
                        car_EMM_Rotate_Clockwise(KeepStraight.out);
                    }
                    else
                    {
                        car_EMM_Rotate_CounterClockwise(-KeepStraight.out);
                    }
//                    count++;
                    if(fabs(straightError) < 0.002f)
                    {
                        beStraightCaliBrated = true;
                        
                        break;
                    }
                }
            }
            else
            {
                if (beStraightCaliBrated)
                {
                    uint32_t targetposition = nowtargetposition - ((uint32_t)(LF_Position[0] << 24 | LF_Position[1] << 16 | LF_Position[2] << 8 | LF_Position[3] << 0) * 3600) / 65536;
                    if(leftward)
                    {
                        
                        car_Move_Leftward_EMM(targetposition);
                        // uint32_t lf = nowtargetposition-  (uint32_t)(LF_Position[0] << 24 | LF_Position[1] << 16 | LF_Position[2] << 8 | LF_Position[3] << 0);
                        // uint32_t lb = nowtargetposition - (uint32_t)(LB_Position[0] << 24 | LB_Position[1] << 16 | LB_Position[2] << 8 | LB_Position[3] << 0);
                        // uint32_t rf = nowtargetposition - (uint32_t)(RF_Position[0] << 24 | RF_Position[1] << 16 | RF_Position[2] << 8 | RF_Position[3] << 0);
                        // uint32_t rb = nowtargetposition - (uint32_t)(RB_Position[0] << 24 | RB_Position[1] << 16 | RB_Position[2] << 8 | RB_Position[3] << 0);
                        // eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Velo & 0xFF00) >> 8);
                        // eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Velo & 0x00FF) >> 0);
                        // eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Acc & 0xFF) >> 0);
                        // motor_EMM_LF_Forward(lf);
                        // motor_EMM_LB_Backward(lb);
                        // motor_EMM_RB_Forward(rf);
                        // motor_EMM_RF_Backward(rb);
                        // motor_Sync_Move();
                    }
                    else
                    {
                        car_Move_Rightward_EMM(targetposition);
                        // uint32_t lf = nowtargetposition - (uint32_t)(LF_Position[0] << 24 | LF_Position[1] << 16 | LF_Position[2] << 8 | LF_Position[3] << 0);
                        // uint32_t lb = nowtargetposition - (uint32_t)(LB_Position[0] << 24 | LB_Position[1] << 16 | LB_Position[2] << 8 | LB_Position[3] << 0);
                        // uint32_t rf = nowtargetposition - (uint32_t)(RF_Position[0] << 24 | RF_Position[1] << 16 | RF_Position[2] << 8 | RF_Position[3] << 0);
                        // uint32_t rb = nowtargetposition - (uint32_t)(RB_Position[0] << 24 | RB_Position[1] << 16 | RB_Position[2] << 8 | RB_Position[3] << 0);
                        // eMM_MOTOR_Cmd[3] = (uint8_t)((emm_Velo & 0xFF00) >> 8);
                        // eMM_MOTOR_Cmd[4] = (uint8_t)((emm_Velo & 0x00FF) >> 0);
                        // eMM_MOTOR_Cmd[5] = (uint8_t)((emm_Acc & 0xFF) >> 0);
                        // motor_EMM_LF_Backward(lf);
                        // motor_EMM_LB_Forward(lb);
                        // motor_EMM_RF_Forward(rf);
                        // motor_EMM_RB_Backward(rb);
                        // motor_Sync_Move();
                    }
                    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
                    osEventFlagsClear(carEventHandle, carReady);
                    osEventFlagsClear(carEventHandle, carGo);
                    beStraightCaliBrated = false;
                }
                else
                {
                    ;
                }
            }
        }
    }
    /* USER CODE END BESTRAIGHT_TASK */
}

/* USER CODE BEGIN Header_BESTRAIGHT_TASK_TASK */
/**
 * @brief Function implementing the BESTRAIGHT_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Heade_SPEED_TASK */
void SPEED_TASK(void *argument)
{
    /* USER CODE BEGIN SPEED_TASK */

    /* Infinite loop */
    for (;;)
    {
        car = osEventFlagsWait(carEventHandle, carGo, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
        {
            // ask_Motor_Speed();

        }
    }
    /* USER CODE END SPEED_TASK */
}

/* USER CODE END Application */
void wait_for_CAR_OK()
{
    // car = osEventFlagsWait(carEventHandle, carGo, osFlagsWaitAll, osWaitForever);
    car = osEventFlagsWait(carEventHandle, carReady, osFlagsWaitAll, osWaitForever);
    clear_Motor_Position();
}


void begin_Rotate()
{
    vTaskResume(CARROTATETASKHandle);
    car = osEventFlagsSet(carEventHandle, rotateGO);
}

void wait_for_Rotate_OK()
{
    car = osEventFlagsWait(carEventHandle, rotateInited, osFlagsWaitAll, osWaitForever);
    car = osEventFlagsWait(carEventHandle, rotateAfter, osFlagsWaitAll, osWaitForever);
    vTaskSuspend(CARROTATETASKHandle);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    clear_Motor_Position();
}

void begin_Calibration()
{
    vTaskResume(CALIBTASKHandle);
    car = osEventFlagsSet(carEventHandle, calibGo);
}

void wait_for_CaliBrated_OK()
{
    car = osEventFlagsWait(carEventHandle, caliBrated, osFlagsWaitAll, osWaitForever);
    vTaskSuspend(CALIBTASKHandle);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    clear_Motor_Position();
}
