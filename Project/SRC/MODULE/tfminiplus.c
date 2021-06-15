//
// Created by Liuyufanlyf on 2021/06/05.
//

#include "string.h"
#include "tfminiplus.h"
#include "board.h"

tfminiplus_data_t tf_data;

void TFminiPlus_init(void) {
    tf_data.length = 0;
    tf_data.singal_strength = 0;
    tf_data.avaliable = false;

    HAL_UART_Receive_DMA(&COM3, tf_data.raw_data, 9);
}

void TFminiPlus_update(void) {
    static uint8_t raw[9];
    memcpy(raw, tf_data.raw_data, 9);

    // Once the first and the second bytes are both right,
    // checksum it's not necessary.
    // So comment out to improve performance
    if (raw[0] == 0x59 && raw[1] == 0x59) {
        // Calculate the checksum
        uint16_t checksum = 0;
        for (uint8_t i = 0; i < 8; i++) {
            checksum += raw[i];
        }
        checksum &= 0xFF;

        if (checksum == raw[8]) {
            tf_data.length = raw[2] + (raw[3] << 8);
            tf_data.singal_strength = raw[4] + (raw[5] << 8);
            if (tf_data.singal_strength > 100 && tf_data.singal_strength < 65535) {
                tf_data.avaliable = true;
                memset(raw, 0, 9);
            } else {
                tf_data.avaliable = false;
                memset(raw, 0, 9);
                return;
            }
        } else {
            tf_data.avaliable = false;
            memset(raw, 0, 9);
            return;
        }
    } else {
        tf_data.avaliable = false;
        memset(raw, 0, 9);
        return;
    }
}

int TFminiPlus_getDistance(void) {
    return tf_data.length;
}

int TFminiPlus_getSignalStrength(void) {
    return tf_data.singal_strength;
}

bool TFminiPlus_getAvaliable(void) {
    bool temp = tf_data.avaliable;
    tf_data.avaliable = false;
    return temp;
}