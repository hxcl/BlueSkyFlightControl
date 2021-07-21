#ifndef _COMMANDCONTROL_H_
#define _COMMANDCONTROL_H_

#include "board.h"

enum
{
    COMMAND_RECEIVED = 0,
    FINISH_ARM_FEEDBACK,
    START_TAKEOFF_FEEDBACK,
    FINISH_TAKEOFF_FEEDBACK,
    FINISH_POSITION_HOLD,
    FINISH_MOVE_X_FEEDBACK,
    FINISH_MOVE_Y_FEEDBACK,
    FINISH_MOVE_Z_FEEDBACK,
    START_LAND_FEEDBACK,
    FINISH_LAND_FEEDBACK,
    FINISH_DISARM_FEEDBACK,

    COMMAND_STARTUP_CHECK
};

typedef struct {
    uint8_t ArmFlag;
    uint8_t DisarmFlag;
    uint8_t TakeOffFlag;
    uint8_t LandFlag;

    uint8_t PositionHoldFlag;
    uint8_t PositionXChangeFlag;
    uint8_t PositionYChangeFlag;
    uint8_t PositionZChangeFlag;

    uint8_t TakeOffAltitude;
    int8_t PositionXChange;
    int8_t PositionYChange;
    int8_t PositionZChange;
} COMMAND_t;

extern COMMAND_t command;

void CommandInit(void);
void CommandDataDecode();

void CommandFeedback(uint8_t feedback);

#endif
