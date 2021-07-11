/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_pwm.c
 * @说明     定时器PWM输出驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "drv_pwm.h"

/**********************************************************************************************************
*函 数 名: PWM_Init
*功能说明: PWM输出初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PWM_Init(void) {
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM8_Init();

    HAL_TIM_PWM_Start(&PWM1_TIM, PWM1_CH);
    HAL_TIM_PWM_Start(&PWM2_TIM, PWM2_CH);
    HAL_TIM_PWM_Start(&PWM3_TIM, PWM3_CH);
    HAL_TIM_PWM_Start(&PWM4_TIM, PWM4_CH);

    HAL_TIM_PWM_Start(&SERVO_PWM1_TIM, SERVO_PWM1_CH);
    HAL_TIM_PWM_Start(&SERVO_PWM2_TIM, SERVO_PWM2_CH);
    HAL_TIM_PWM_Start(&SERVO_PWM3_TIM, SERVO_PWM3_CH);
    HAL_TIM_PWM_Start(&SERVO_PWM4_TIM, SERVO_PWM4_CH);
}

/**********************************************************************************************************
*函 数 名: TempControlPWMSet
*功能说明: 温控PWM输出值设置
*形    参: PWM值（0-1000）
*返 回 值: 无
**********************************************************************************************************/
void TempControlPWMSet(int32_t pwmValue) {
    pwmValue = pwmValue * ((float) TEMP_TIM_PERIOD / 1000.f);

    __HAL_TIM_SET_COMPARE(&TEMP_TIM, TEMP_CH, pwmValue);
}

/**********************************************************************************************************
*函 数 名: ServoControlPWMSet
*功能说明: 舵机PWM输出值设置
*形    参: Servo number & PWM value（0-2000）
*返 回 值: 无
**********************************************************************************************************/
void ServoControlPWMSet(uint8_t motor, int32_t pwmValue) {
    pwmValue = pwmValue / 2 + 1000;

    if (motor == 1) {
        __HAL_TIM_SET_COMPARE(&SERVO_PWM1_TIM, SERVO_PWM1_CH, pwmValue);
    } else if (motor == 2) {
        __HAL_TIM_SET_COMPARE(&SERVO_PWM2_TIM, SERVO_PWM2_CH, pwmValue);
    } else if (motor == 3) {
        __HAL_TIM_SET_COMPARE(&SERVO_PWM3_TIM, SERVO_PWM3_CH, pwmValue);
    } else if (motor == 4) {
        __HAL_TIM_SET_COMPARE(&SERVO_PWM4_TIM, SERVO_PWM4_CH, pwmValue);
    }
}


/**********************************************************************************************************
*函 数 名: MotorPWMSet
*功能说明: 电机PWM输出值设置
*形    参: 电机号 PWM值（0-2000）
*返 回 值: 无
**********************************************************************************************************/
void MotorPWMSet(uint8_t motor, uint16_t pwmValue) {

#if(ESC_PROTOCOL == PWM)
    pwmValue = 5 * pwmValue + 8000;
#endif

    if (motor == 1) {
        __HAL_TIM_SET_COMPARE(&PWM1_TIM, PWM1_CH, pwmValue);
    } else if (motor == 2) {
        __HAL_TIM_SET_COMPARE(&PWM2_TIM, PWM2_CH, pwmValue);
    } else if (motor == 3) {
        __HAL_TIM_SET_COMPARE(&PWM3_TIM, PWM3_CH, pwmValue);
    } else if (motor == 4) {
        __HAL_TIM_SET_COMPARE(&PWM4_TIM, PWM4_CH, pwmValue);
    }
}