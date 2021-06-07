//
// Created by 12484 on 2021/06/05.
//

#ifndef QUADCOPTER_TFMINIPLUS_H
#define QUADCOPTER_TFMINIPLUS_H

#include "stdbool.h"

#include "usart.h"

typedef struct {
    int length;
    int singal_strength;
    bool avaliable;
} tfminiplus_data_t;

extern tfminiplus_data_t tf_data;

void TFminiPlus_init(void);

void TFminiPlus_update(uint8_t *raw);

int TFminiPlus_getDistance(void);

int TFminiPlus_getSignalStrength(void);

bool TFminiPlus_getAvaliable(void);

#endif //QUADCOPTER_TFMINIPLUS_H
