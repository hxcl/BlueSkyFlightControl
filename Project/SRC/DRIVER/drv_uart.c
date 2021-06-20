/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_uart.c
 * @说明     串口驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include <usbd_cdc_if.h>
#include <bsklinkDecode.h>
#include "drv_uart.h"
#include "string.h"
#include "drv_sbus.h"
#include "tfminiplus.h"

uint8_t COM1TxBuf[256];
uint8_t COM2TxBuf[256];
uint8_t COM3TxBuf[256];
uint8_t COM4TxBuf[256];
uint8_t COM5TxBuf[256];

uint8_t COM1RxBuf[32];
uint8_t COM2RxBuf[32];
uint8_t COM3RxBuf[32];
uint8_t COM4RxBuf[32];
uint8_t COM5RxBuf[32];

/**********************************************************************************************************
*函 数 名: Uart_Open
*功能说明: 串口初始化
*形    参: 串口号
*返 回 值: 无
**********************************************************************************************************/
void Uart_Init() {
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_UART4_Init();
    MX_UART7_Init();
    MX_UART8_Init();

    // bsklink/Mavlink via COM1(huart1)
    HAL_UART_Receive_DMA(&COM1, COM1RxBuf, 1);

    // Copter computer communication via COM2(huart3)
    HAL_UART_Receive_IT(&COM2, COM2RxBuf, 32);

    // S.BUS via COM5(huart8)
    HAL_UART_Receive_IT(&COM5, COM5RxBuf, 25);
}

static UartCallback uart1CallbackFunc;
static UartCallback uart2CallbackFunc;
static UartCallback uart3CallbackFunc;
static UartCallback uart4CallbackFunc;
static UartCallback uart5CallbackFunc;

/**********************************************************************************************************
*函 数 名: Usart_SetIRQCallback
*功能说明: 设置串口接收中断回调函数
*形    参: 串口号 回调函数
*返 回 值: 无
**********************************************************************************************************/
void Uart_SetIRQCallback(uint8_t deviceNum, UartCallback uartCallback) {
    if (deviceNum == 1) {
        uart1CallbackFunc = uartCallback;
    } else if (deviceNum == 2) {
        uart2CallbackFunc = uartCallback;
    } else if (deviceNum == 3) {
        uart3CallbackFunc = uartCallback;
    } else if (deviceNum == 4) {
        uart4CallbackFunc = uartCallback;
    } else if (deviceNum == 5) {
        uart5CallbackFunc = uartCallback;
    }
}

/**********************************************************************************************************
*函 数 名: Uart_SendData
*功能说明: 串口数据发送函数
*形    参: 串口号 发送数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void Uart_SendData(uint8_t deviceNum, uint8_t *DataToSend, uint8_t length) {
    if (deviceNum == 1) {
        memcpy(&COM1TxBuf, DataToSend, length);
        HAL_UART_Transmit(&COM1, COM1TxBuf, length, 100);
    } else if (deviceNum == 2) {
        memcpy(&COM2TxBuf, DataToSend, length);
        HAL_UART_Transmit_DMA(&COM2, COM2TxBuf, length);
    } else if (deviceNum == 3) {
        memcpy(&COM3TxBuf, DataToSend, length);
        HAL_UART_Transmit_DMA(&COM3, COM3TxBuf, length);
    } else if (deviceNum == 4) {
        memcpy(&COM4TxBuf, DataToSend, length);
        HAL_UART_Transmit_DMA(&COM4, COM4TxBuf, length);
    } else if (deviceNum == 5) {
        memcpy(&COM5TxBuf, DataToSend, length);
        HAL_UART_Transmit_DMA(&COM5, COM5TxBuf, length);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &COM1) {
        BsklinkDecode(COM1RxBuf[0]);
        //HAL_UART_Receive_IT(&huart1, COM1RxBuf, 1);
    } else if (huart == &COM2) {

        HAL_UART_Receive_IT(&COM2, COM2RxBuf, 9);
    } else if (huart == &COM5) {
        for (uint8_t i = 0; i < 25; i++) {
            Sbus_Decode(COM5RxBuf[i]);
        }
        HAL_UART_Receive_IT(&COM5, COM5RxBuf, 25);
    }
}
