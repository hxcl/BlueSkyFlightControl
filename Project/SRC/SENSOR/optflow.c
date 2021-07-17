//
// Created by Liuyufanlyf on 2021/06/14.
//

#include "optflow.h"
#include "string.h"

#include "boardConfigBlueSkyV3.h"
#include "gyroscope.h"
#include "ToF_altimeter.h"
#include "navigation.h"
#include "flightStatus.h"
#include "LC302.h"
#include "PX4FLOW.h"
#include "board.h"
#include "lowPassFilter.h"

#define USE_TOF_ALTITUDE

#define LPF_1_(hz, t, in, out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

typedef struct {
    float time_s;
    bool update_flag;
    bool enable_lpf;

    float Ref_gnd_vel_x;
    float Ref_gnd_vel_y;

    float Velocity_x;
    float Velocity_y;

    float Velocity_x_lpf;
    float Velocity_y_lpf;
    float Gyro_x;
    float Gyro_y;
    float Gyro_x_phase;
    float Gyro_y_phase;

    float Velocity_uncoupled_x;
    float Velocity_uncoupled_y;

    float Velocity_uncoupled_lpf_x;
    float Velocity_uncoupled_lpf_y;

    float Position_x;
    float Position_y;

    float Gnd_Position_x;
    float Gnd_Position_y;
} optflow_manager_t;

optflow_manager_t optflow_manager;

//#define OPT_FLOW_PLACE_BACK_LINE

void OptFlowDataTreat(void) {
    static uint64_t lastTime = 0;

    static uint16_t pos_reset_cnt = 0;

    static float gyro_phase_x = 0;
    static float gyro_phase_y = 0;

    float temp_x_flow, temp_y_flow;

    if (OPTFLOW_TYPE == LC302) {
        if (LC302_getAvaliable()) {
            temp_x_flow = (float) LC302_get_Y_Integral();
            temp_y_flow = (float) LC302_get_X_Integral();
            optflow_manager.update_flag = true;
            pos_reset_cnt++;
        } else {
            optflow_manager.update_flag = false;
        }

        float Height = ConstrainFloat(GetCopterPosition().z, 0, 500);

        // 角速度
        optflow_manager.Velocity_x = -1 * temp_x_flow / 0.0208 / 10000.f;
        optflow_manager.Velocity_y = temp_y_flow / 0.0208 / 10000.f;

        // 10Hz 低通，延时 0.02s
        optflow_manager.Velocity_x_lpf =
                0.55674 * optflow_manager.Velocity_x + 0.44326 * optflow_manager.Velocity_x_lpf;
        optflow_manager.Velocity_y_lpf =
                0.55674 * optflow_manager.Velocity_y + 0.44326 * optflow_manager.Velocity_y_lpf;

        optflow_manager.Gyro_x = ConstrainFloat(Radians(GyroGetData().x), -100, 100);
        optflow_manager.Gyro_y = ConstrainFloat(Radians(GyroGetData().y), -100, 100);

        // 4Hz 低通，延时 0.02s
        optflow_manager.Gyro_x_phase = (optflow_manager.Gyro_x * 0.334398 + optflow_manager.Gyro_x_phase * 0.665602);
        optflow_manager.Gyro_y_phase = (optflow_manager.Gyro_y * 0.334398 + optflow_manager.Gyro_y_phase * 0.665602);

        //陀螺仪解耦合，单位是像素Pix单位时间内像素的位移
        optflow_manager.Velocity_uncoupled_x = optflow_manager.Velocity_x_lpf + optflow_manager.Gyro_x_phase;
        optflow_manager.Velocity_uncoupled_y = optflow_manager.Velocity_y_lpf - optflow_manager.Gyro_y_phase;

        //光流速度限幅
        optflow_manager.Velocity_uncoupled_x = ConstrainFloat(optflow_manager.Velocity_uncoupled_x, -100.0f, 100.0f);
        optflow_manager.Velocity_uncoupled_y = ConstrainFloat(optflow_manager.Velocity_uncoupled_y, -100.0f, 100.0f);

        optflow_manager.Velocity_uncoupled_lpf_x =
                optflow_manager.Velocity_uncoupled_lpf_x * 0.9 + optflow_manager.Velocity_uncoupled_x * 0.1;
        optflow_manager.Velocity_uncoupled_lpf_y =
                optflow_manager.Velocity_uncoupled_lpf_y * 0.9 + optflow_manager.Velocity_uncoupled_y * 0.1;

        optflow_manager.Ref_gnd_vel_x = optflow_manager.Velocity_uncoupled_lpf_x * Height;
        optflow_manager.Ref_gnd_vel_y = optflow_manager.Velocity_uncoupled_lpf_y * Height;

        //累计位移
        optflow_manager.Gnd_Position_x += optflow_manager.Ref_gnd_vel_x * 0.0208;
        optflow_manager.Gnd_Position_y += optflow_manager.Ref_gnd_vel_y * 0.0208;

        optflow_manager.update_flag = true;
    } else if (OPTFLOW_TYPE == PX4FLOW) {
        PX4FLOW_Integral_Update();
        if (PX4FLOW_GetQuality_Integral() > 100) {
            temp_x_flow = (float) PX4FLOW_GetPixelFlowX_Integral() / 10.f;
            temp_y_flow = (float) PX4FLOW_GetPixelFlowY_Integral() / 10.f;
            gyro_phase_x = (float) PX4FLOW_GetGyroX_Integral() / 10.f;
            gyro_phase_y = (float) PX4FLOW_GetGyroY_Integral() / 10.f;

            temp_x_flow = temp_x_flow + gyro_phase_x;
            temp_y_flow = temp_y_flow + gyro_phase_y;

            uint32_t timespan = PX4FLOW_GetTimestamp_Integral();

#ifdef USE_TOF_ALTITUDE
            // mm
            float ground_distance = (float) ToFAltimeterGetAlt() * 10.f;
#else
            // 当前应用场景不太可能有超过 400cm 的高度，此时认为声呐高度失效
            int ground_distance = PX4FLOW_GetGroundDistance_Integral();
            if (ground_distance >= 400) {
                ground_distance = ToFAltimeterGetAlt() * 10.f;
            }
#endif
            // 100 * mrad * mm / us = cm
            optflow_manager.Ref_gnd_vel_x = -100 * temp_x_flow * ground_distance / 10000;     // cm/s
            optflow_manager.Ref_gnd_vel_y = 100 * temp_y_flow * ground_distance / 10000;     // cm/s

            // Integrate velocity to get pose estimate
            // cm
            optflow_manager.Gnd_Position_x += optflow_manager.Ref_gnd_vel_x / 100.f;
            optflow_manager.Gnd_Position_y += optflow_manager.Ref_gnd_vel_y / 100.f;
        }
    }
}

float OptFlowGetGroundPositionX(void) {
    return optflow_manager.Gnd_Position_x;
}

float OptFlowGetGroundPositionY(void) {
    return optflow_manager.Gnd_Position_y;
}

float OptFlowGetGroundVelocityX(void) {
    return optflow_manager.Ref_gnd_vel_x;
}

float OptFlowGetGroundVelocityY(void) {
    return optflow_manager.Ref_gnd_vel_y;
}

void OptFlowClearGroundPosition(void) {
    optflow_manager.Gnd_Position_x = 0;
    optflow_manager.Gnd_Position_y = 0;
}