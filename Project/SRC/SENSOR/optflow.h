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

int OptFlowGetGroundPositionX(void);

int OptFlowGetGroundPositionY(void);

int OptFlowGetGroundVelocityX(void);

int OptFlowGetGroundVelocityY(void);

void OptFlowClearGroundPosition(void);

#endif //OPTFLOW_H
