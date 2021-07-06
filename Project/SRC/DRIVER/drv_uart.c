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

    // ToF sensor communication via COM2(huart3)
    HAL_UART_Receive_DMA(&COM2, COM2RxBuf, 1);

    // S.BUS via COM5(huart8)
    HAL_UART_Receive_IT(&COM5, COM5RxBuf, 1);
}

/**********************************************************************************************************
*函 数 名: Uart_SendData
*功能说明: 串口数据发送函数
*形    参: 串口号 发送数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void Uart_SendData(uint8_t deviceNum, uint8_t *DataToSend, uint8_t length) {
    if (deviceNum == 1) {
        HAL_UART_Transmit_DMA(&COM1, DataToSend, length);
        //HAL_UART_Transmit(&COM1, DataToSend, length, 100);
    } else if (deviceNum == 2) {
        HAL_UART_Transmit_DMA(&COM2, DataToSend, length);
    } else if (deviceNum == 3) {
        HAL_UART_Transmit_DMA(&COM3, DataToSend, length);
    } else if (deviceNum == 4) {
        HAL_UART_Transmit_DMA(&COM4, DataToSend, length);
    } else if (deviceNum == 5) {
        HAL_UART_Transmit_DMA(&COM5, DataToSend, length);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &COM1) {
        BsklinkDecode(COM1RxBuf[0]);
        //HAL_UART_Receive_IT(&huart1, COM1RxBuf, 1);
    } else if (huart == &COM2) {
        TFminiPlus_Decode(COM2RxBuf[0]);
        //HAL_UART_Receive_IT(&COM2, COM2RxBuf, 1);
    } else if (huart == &COM5) {
        Sbus_Decode(COM5RxBuf[0]);
        HAL_UART_Receive_IT(&COM5, COM5RxBuf, 1);
    }
}
