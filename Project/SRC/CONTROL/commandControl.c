#include "commandControl.h"
#include "flightStatus.h"
#include "drv_uart.h"

COMMAND_t command;

uint8_t CommandFeedbackBuffer[4];

void CommandInit(void){
    CommandFeedbackBuffer[0] = 0xFE;
    CommandFeedbackBuffer[2] = 0xFE;
    CommandFeedbackBuffer[3] = 0xFE;

    CommandFeedback(COMMAND_STARTUP_CHECK);
}

void CommandDataDecode(uint8_t *raw){

}

void CommandFeedback(uint8_t feedback){
    CommandFeedbackBuffer[1] = feedback;
    Uart_SendData(6, (uint8_t *)CommandFeedbackBuffer, 4);
}