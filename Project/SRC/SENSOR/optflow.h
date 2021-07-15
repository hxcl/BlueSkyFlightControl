//
// Created by Liuyufanlyf on 2021/06/14.
//

#ifndef OPTFLOW_H
#define OPTFLOW_H

#include "stdint.h"
#include "stdbool.h"
#include "mathTool.h"

void OptFlowDataTreat(void);

void OptFlowDataClear(void);

float OptFlowGetGroundPositionX(void);

float OptFlowGetGroundPositionY(void);

float OptFlowGetGroundVelocityX(void);

float OptFlowGetGroundVelocityY(void);

void OptFlowClearGroundPosition(void);

#endif //OPTFLOW_H
