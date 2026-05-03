/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights rx_Buffer_Servoerved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include "stdio.h"
#include "string.h"
#include "task.h"
#include "led.h"
#include "servo.h"
#include "stepmotor.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "cmsis_os.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum
{
    UART1_RX_ERROR,
    UART4_RX_ERROR,

} _ERROR;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile unsigned long CPU_RunTime = 0UL;

uint32_t uart4_flag;

_ERROR Error;

uint8_t rx_Buffer_IMU[BUFF_SIZE];

uint8_t rx_Buffer_Stepmotor[20];
uint8_t rx_Buffer_AUX[20];


uint8_t rx_Buffer_Openmv[10];
uint8_t openmv_Buffer[10];
uint8_t openmv_QR_Buffer[10];

uint8_t LF_Position[4];
uint8_t RF_Position[4];
uint8_t LB_Position[4];
uint8_t RB_Position[4];
uint8_t position_count = 0;

float center_x;
float center_y;

uint8_t rx_Buffer_Servo[7];
uint8_t rx_Buffer_Servo_size = 2;

static bool isGotFrameHeader = false;


extern osSemaphoreId_t IMU_Bin_SemaphoreHandle;

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern uint8_t Fd_rsimu[64];
extern uint8_t Fd_rsahrs[56];

//uint32_t data_length;
extern uint8_t rs_ahrstype;
extern uint8_t rs_imutype;

extern osEventFlagsId_t positionEventHandle;
extern osEventFlagsId_t carEventHandle;
extern osEventFlagsId_t armEventHandle;
extern uint32_t arm;
int16_t carcount=0;
int16_t carcount1=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//   LF_Buffer = (uint8_t *)pvPortMalloc(4 * sizeof(uint8_t));
//   RF_Buffer = (uint8_t *)pvPortMalloc(4 * sizeof(uint8_t));
//   LB_Buffer = (uint8_t *)pvPortMalloc(4 * sizeof(uint8_t));
//   RB_Buffer = (uint8_t *)pvPortMalloc(4 * sizeof(uint8_t));

//   MOVE_Buffer = (uint8_t *)pvPortMalloc(4 * sizeof(uint8_t));


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	led_red_off();
	led_blue_off();

//   motor_Disable();
//   while(信号量 按键)
//  motor_Enabled();

    /*
        CPU TASK 可以注释
    */
    // HAL_TIM_Base_Start_IT(&htim4);
    /*
        初始化动作组
    */
    motor_DisResponce();
    openMV_Work_Mode(0);
    runActionGroup(action_armStandby, 1);
    HAL_Delay(2000);
		clear_Motor_Position();
        

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_Buffer_Stepmotor, BUFF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_Openmv, BUFF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_Buffer_AUX, BUFF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_Buffer_IMU, BUFF_SIZE);

        //    runActionGroup(action_armStandby, 1);
        //    HAL_Delay(2000);

        HAL_UART_Receive_IT(&huart5, rx_Buffer_Servo, rx_Buffer_Servo_size);

        led_blue_on();

        /* USER CODE END 2 */

        /* Init scheduler */
        osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
        MX_FREERTOS_Init();

        /* Start scheduler */
        osKernelStart();
        /* We should never get here as control is now taken by the scheduler */
        /* Infinite loop */
        /* USER CODE BEGIN WHILE */

        while (1)
        {
            /* USER CODE END WHILE */

            /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

int16_t data_length = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4)
    {
//		uint32_t a;
		
        HAL_UART_DMAStop(&huart4); // 把DMA接收停掉，防止DMA接收速度过快导致中断重入，数据被覆写
        data_length = BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
        if (rs_ahrstype == 0 && data_length == 56)
        {
            memcpy(Fd_rsahrs, rx_Buffer_IMU, data_length);
            
            osEventFlagsSet(carEventHandle, imuRawDataReceived);
					
            rs_ahrstype = 1;
        }
        // ??? delete??? 优化性能？ 
        if (rs_imutype == 0 && data_length == 64)
        {
            memcpy(Fd_rsimu, rx_Buffer_IMU, data_length);
//            osEventFlagsSet(carEventHandle, accRawDataReceived);
            rs_imutype = 1;
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_Buffer_IMU, BUFF_SIZE); // 接收完毕后重开空闲DMA接收
        memset(rx_Buffer_IMU, 0, data_length);
    }
/*************************************************************************************************/
    if (huart->Instance == USART2)
    {
        // check baudrate 9600 communication with openmv
         int16_t data_length = 0;
        HAL_UART_DMAStop(&huart2); // 把DMA接收停掉，防止DMA接收速度过快导致中断重入，数据被覆写
        data_length = BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
        if (data_length == 10)                                       // 二维码
        {
            if ((rx_Buffer_Openmv[0] == '&') && (rx_Buffer_Openmv[8] == '*') && (rx_Buffer_Openmv[9] == '/'))
            {
                memcpy(openmv_QR_Buffer, &rx_Buffer_Openmv[1], 7);
                osEventFlagsSet(carEventHandle, taskCodeGot);
            }
        }
        else if (data_length == 4) // 颜色
        {
            if ((rx_Buffer_Openmv[0] == 0x06) && (rx_Buffer_Openmv[2] == 0x09) && (rx_Buffer_Openmv[3] == 0x0A))
            {
                switch(rx_Buffer_Openmv[1])
                {
                    case 0x00:
                        osEventFlagsSet(carEventHandle, redDetected);
                        break;
                    case 0x01:
                        osEventFlagsSet(carEventHandle, greenDetected);
                        break;
                    case 0x02:
                        osEventFlagsSet(carEventHandle, blueDetected);
                        break;
                }
            }
            memset(rx_Buffer_Openmv, 0, 4);
        }
        else if (data_length == 8) // 中心坐标
        {
            if ((rx_Buffer_Openmv[0] == 0x06) && (rx_Buffer_Openmv[6] == 0x09) && (rx_Buffer_Openmv[7] == 0x0A))
            {
                //            memcpy(openmv_Buffer, rx_Buffer_Openmv, data_length);
                center_x = (float)(rx_Buffer_Openmv[1] + rx_Buffer_Openmv[2] + (float)rx_Buffer_Openmv[3] / 100);
                center_y = (float)(rx_Buffer_Openmv[4] + (float)rx_Buffer_Openmv[5] / 100);
                osEventFlagsSet(carEventHandle, xyGot);
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_Openmv, BUFF_SIZE); // 接收完毕后重开空闲DMA接收
        memset(rx_Buffer_Openmv, 0, data_length);
        
        // osEventFlagsSet(positionEventHandle, cameraReady);
    }
/*************************************************************************************************/
    if (huart->Instance == USART1)
    {
        int16_t data_length =0;
//        osStatus_t RotateBusy_flag;
        HAL_UART_DMAStop(&huart1); // 把DMA接收停掉，防止DMA接收速度过快导致中断重入，数据被覆写
        data_length = BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        if(data_length==4)
        {
            switch(rx_Buffer_Stepmotor[1])
            {
                case EMM_Cmd:
                    switch (rx_Buffer_Stepmotor[0])
                    {
                        
                        case 0x01:
                            // 运动到位
							// carcount++;
                            osEventFlagsSet(carEventHandle, carReady);
                            // osEventFlagsClear(carEventHandle, carGo);
                            break;
                        default:
                            while(1)
                            {
                                led_red_toggle();
                                osDelay(100);
                            }
                            break;
                    }
                break;
                case SYNC_Cmd:
                    osEventFlagsSet(carEventHandle, carGo);
                    // carcount1++;
                    //出发 收到指令 
                    break;
                default:
    //                while (1)
    //                {
    //                    led_red_toggle();
    //                    osDelay(100);
    //                }
                    break;
            }
        }
        if (data_length == 8)
        {
            switch (rx_Buffer_Stepmotor[0])
            {
            case LF:
                memcpy(LF_Position, &rx_Buffer_Stepmotor[3], 4);
                osEventFlagsSet(carEventHandle, positionGot);
                // position_count++;
                break;
            // case RF:
            //     memcpy(RF_Position, &rx_Buffer_Stepmotor[3], 4);
            //     osEventFlagsSet(carEventHandle, positionChange);
            //     position_count++;
            //     break;
            // case LB:
            //     memcpy(LB_Position, &rx_Buffer_Stepmotor[3], 4);
            //     osEventFlagsSet(carEventHandle, positionChange);
            //     position_count++;
            //     break;
            // case RB:
            //     memcpy(RB_Position, &rx_Buffer_Stepmotor[3], 4);
            //     osEventFlagsSet(carEventHandle, positionChange);
            //     position_count++;
            //     break;
            }
            // if(position_count==4)
            // {
            //     osEventFlagsSet(carEventHandle, positionGot);
            //     position_count = 0;
            // }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_Buffer_Stepmotor, BUFF_SIZE); // 接收完毕后重开空闲DMA接收
        memset(rx_Buffer_Stepmotor, 0, data_length);
    }
/*************************************************************************************************/
    if (huart->Instance == USART3)
    {
        int16_t data_length =0;
        HAL_UART_DMAStop(&huart3); // 把DMA接收停掉，防止DMA接收速度过快导致中断重入，数据被覆写
        data_length = BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        // if(data_length==4)
        // {
            switch(rx_Buffer_AUX[1])
            {
                case EMM_Cmd:
                    switch (rx_Buffer_AUX[0])
                    {
                        
                        case OBJ:
                            // 运动到位
                            osEventFlagsSet(carEventHandle, oBJReady);
                            break;
                        case ARM:
                            osEventFlagsSet(carEventHandle, aRMReady);
                            break;
                        default:
                            while(1)
                            {
                                led_red_toggle();
                                osDelay(100);
                            }
                            break;
                    }
                break;
            default:
//                while (1)
//                {
//                    led_red_toggle();
//                    osDelay(100);
//                }
                break;
            }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_Buffer_AUX, BUFF_SIZE); // 接收完毕后重开空闲DMA接收
        memset(rx_Buffer_AUX, 0, data_length);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    { // 判断接收中断
        if (rx_Buffer_Servo_size == 2)
        {
            if (!isGotFrameHeader)
            { // 判断帧头
                if ((rx_Buffer_Servo[0] == FRAME_HEADER) && (rx_Buffer_Servo[1] == FRAME_HEADER))
                {
                    isGotFrameHeader = true;
                    rx_Buffer_Servo_size = 5;
                    HAL_UART_Receive_IT(&huart5, rx_Buffer_Servo, rx_Buffer_Servo_size);
                }
                else
                {
                    isGotFrameHeader = false;
                    rx_Buffer_Servo_size = 2;
                    HAL_UART_Receive_IT(&huart5, rx_Buffer_Servo, rx_Buffer_Servo_size);
                }
            }
        }
        else if (rx_Buffer_Servo_size == 5)
        {
            if (isGotFrameHeader)
            { // 接收接收数据部分
                if (rx_Buffer_Servo[1]==0x08)   
                {
                    arm = osEventFlagsSet(carEventHandle, armFinish);
									
                }
                isGotFrameHeader = false;
                rx_Buffer_Servo_size = 2;
                HAL_UART_Receive_IT(&huart5, rx_Buffer_Servo, rx_Buffer_Servo_size);
            }
        }
        else
        {
            rx_Buffer_Servo_size = 2;
            HAL_UART_Receive_IT(&huart5, rx_Buffer_Servo, rx_Buffer_Servo_size);
        }
    }	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   if (HAL_UART_GetError(huart) & HAL_UART_ERROR_PE)
   { /*!< Parity error            */
       // 奇偶校验错误
       __HAL_UART_CLEAR_PEFLAG(huart);
   }
   else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_NE)
   { /*!< Noise error             */
       // 噪声错误
       __HAL_UART_CLEAR_NEFLAG(huart);
   }
   else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_FE)
   { /*!< Frame error             */
       // 帧格式错误
       __HAL_UART_CLEAR_FEFLAG(huart);
   }
   else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
   { /*!< Overrun error           */
       // 数据太多串口来不及接收错误
       __HAL_UART_CLEAR_OREFLAG(huart);
   }

   if (huart->Instance == USART1)
   {
    //    memset(rx_Buffer_Stepmotor, 0, BUFF_SIZE);
       HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_Buffer_Stepmotor, BUFF_SIZE); // 接收完毕后重开空闲DMA接收
   }
   if (huart->Instance == USART2)
   {
    //    memset(rx_Buffer_Openmv, 0, BUFF_SIZE);
       HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_Openmv, 10); // 接收完毕后重开空闲DMA接收
   }
   if (huart->Instance == UART4)
   {
    //    memset(rx_Buffer_IMU, 0, BUFF_SIZE);
       HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_Buffer_IMU, BUFF_SIZE); // 接收完毕后重开空闲DMA接收
   }
   if (huart->Instance == UART5)
   {
    //    memset(rx_Buffer_Servo, 0, 7);
       HAL_UART_Receive_IT(&huart5, rx_Buffer_Servo, 2); // 接收完毕后重开空闲DMA接收
   }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM4) {
		 CPU_RunTime++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
