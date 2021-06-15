//
// Created by Liuyufanlyf on 2021/06/05.
//

#ifndef TFMINIPLUS_H
#define TFMINIPLUS_H

#include "stdbool.h"

#include "usart.h"

typedef struct {
    int length;
    int singal_strength;
    uint8_t raw_data[9];
    bool avaliable;
} tfminiplus_data_t;

extern tfminiplus_data_t tf_data;

void TFminiPlus_init(void);

void TFminiPlus_update(void);

int TFminiPlus_getDistance(void);

int TFminiPlus_getSignalStrength(void);

bool TFminiPlus_getAvaliable(void);

#endif //TFMINIPLUS_H
