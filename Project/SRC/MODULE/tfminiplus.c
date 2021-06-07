//
// Created by 12484 on 2021/06/05.
//

#include "tfminiplus.h"

tfminiplus_data_t tf_data;

void TFminiPlus_init(void) {
    tf_data.length = 0;
    tf_data.singal_strength = 0;
    tf_data.avaliable = false;
}

void TFminiPlus_update(uint8_t *raw) {

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
            } else {
                tf_data.avaliable = false;
                return;
            }
        } else {
            tf_data.avaliable = false;
            return;
        }
    } else {
        tf_data.avaliable = false;
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