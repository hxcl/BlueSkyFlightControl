//
// Created by liuyufanlyf on 2021/06/15.
//

#include "LC302.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "drv_uart.h"

#pragma pack (1)
typedef struct {
    uint8_t frame_head;
    uint8_t frame_length;
    int16_t flow_x_integral;
    int16_t flow_y_integral;
    uint16_t integration_timespan;
    uint16_t ground_distance;
    uint8_t valid;
    uint8_t version;
    uint8_t check;
    uint8_t frame_tail;

    uint8_t raw_data[14];
    bool available
} lc302_t;
#pragma pack ()

lc302_t lc302;

void LC302_init(void) {
    HAL_UART_Receive_DMA(&COM4, lc302.raw_data, 14);
}

void LC302_update(void) {
    static uint8_t data_temp[14];
    uint8_t XOR = 0;

    memcpy(data_temp, lc302.raw_data, 14);

    for (int i = 2; i <= 11; i++) {
        XOR ^= data_temp[i];
    }

    if (XOR == data_temp[12]) {
        lc302.frame_head = data_temp[0];
        lc302.frame_length = data_temp[1];
        lc302.flow_x_integral = data_temp[2] + data_temp[3] << 8;
        lc302.flow_y_integral = data_temp[4] + data_temp[5] << 8;
        lc302.integration_timespan = data_temp[6] + data_temp[7] << 8;
        lc302.ground_distance = data_temp[8] + data_temp[9] << 8;
        lc302.valid = data_temp[10];
        lc302.version = data_temp[11];
        lc302.check = data_temp[12];
        lc302.frame_tail = data_temp[13];

        //如果valid值为0xF5，则说明此帧有效
        if (lc302.valid != 0xF5) {
            lc302.flow_x_integral = 0;
            lc302.flow_y_integral = 0;
            lc302.available = false;
        }
        else {
        lc302.available = true;
        }
    }
    else{
        lc302.available = false;
    }

    memset(data_temp, 0, 14);
}

int LC302_get_X_Integral(void){
    return lc302.flow_y_integral;
}

int LC302_get_Y_Integral(void){
    return lc302.flow_y_integral;
}

bool LC302_getAvaliable(void){
    return lc302.available;
}


