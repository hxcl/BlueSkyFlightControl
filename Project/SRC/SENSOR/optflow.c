//
// Created by Liuyufanlyf on 2021/06/14.
//

#include "optflow.h"
#include "string.h"
#include "gyroscope.h"
#include "navigation.h"
#include "flightStatus.h"
#include "LC302.h"
#include "board.h"

typedef struct {
    float time_s;
    bool update_flag;
    bool enable_lpf;

    float Ref_height;
    float Ref_gnd_vel_x;
    float Ref_gnd_vel_y;

    float flow_dat_x;
    float flow_dat_y;

    float Velocity_x;
    float Velocity_y;

    float Velocity_use_x;
    float Velocity_use_y;

    float Velocity_x_lpf;
    float Velocity_y_lpf;
    float Gyro_x;
    float Gyro_y;
    float Gyro_x_phase;
    float Gyro_y_phase;

    float Velocity_uncoupled_x[2];
    float Velocity_uncoupled_y[2];

    float tmp;

    float Acc_x;
    float Acc_y;

    float Position_x;
    float Position_y;

    float Gnd_Position_x;
    float Gnd_Position_y;

    float pos_x;
    float pos_y;
} up_optflow_manager_t;

up_optflow_manager_t up_optflow_manager;

//#define OPT_FLOW_PLACE_BACK_LINE

void OptflowDataTreat(void) {
    static uint64_t lastTime = 0;
    int32_t ToFAltTemp;

    static float xielv_tmp = 0;

    static uint16_t pos_reset_cnt = 0;

    static float Last_Velo_x = 0;
    static float Last_Velo_y = 0;

    static float acc_tmp_x = 0;
    static float acc_tmp_y = 0;

    static float tmp_x = 0;
    static float tmp_y = 0;

    static float UP_OptFlow_Lpf_X_tmp = 0;
    static float UP_OptFlow_Lpf_Y_tmp = 0;

    static float gyro_phase_x = 0;
    static float gyro_phase_y = 0;

    static float compensation_gyro_y = 0;

    int temp_x_flow, temp_y_flow;

    float deltaT = (GetSysTimeUs() - lastTime) * 1e-6;
    lastTime = GetSysTimeUs();

    LC302_update();
    if (LC302_getAvaliable()) {
        temp_x_flow = LC302_get_X_Integral();
        temp_y_flow = LC302_get_Y_Integral();
        up_optflow_manager.update_flag = true;
        pos_reset_cnt++;
    } else {
        up_optflow_manager.update_flag = false;
    }
    //反向Y轴数据
    temp_x_flow *= -1;

    //20.8ms = 0.02s
    up_optflow_manager.time_s = 0.02f;

    //第一步：根据官方给出的计算公式计算速度
    //官方给出的速度计算方法 speed_x = (h*flow_dat.x / (integration_timespan * 0.000001))*100
    //高度扩大10倍，表示单位是mm
    up_optflow_manager.Ref_height = ConstrainFloat(GetCopterPosition().z, 0, 500);

#ifdef OPT_FLOW_PLACE_BACK_LINE
    //临时修改，此处适用光流线朝后的情况
    //单位：rad/s
    up_optflow_manager.Velocity_y = -up_optflow_manager.up_optflow.flow_x_integral / up_optflow_manager.time_s / 10000.0f;
    up_optflow_manager.Velocity_x = -up_optflow_manager.up_optflow.flow_y_integral / up_optflow_manager.time_s / 10000.0f;

#else
    //flow_x单位：像素               像素积分 / 时间 = 积分/s
    up_optflow_manager.Velocity_x = temp_x_flow / up_optflow_manager.time_s / 10000.0f;
    up_optflow_manager.Velocity_y = temp_y_flow / up_optflow_manager.time_s / 10000.0f;
#endif

    //lpf
    //LPF_1_(15, 0.02f, up_optflow_manager.Velocity_x, up_optflow_manager.Velocity_x_lpf);
    //LPF_1_(15, 0.02f, up_optflow_manager.Velocity_y, up_optflow_manager.Velocity_y_lpf);

    up_optflow_manager.Gyro_x = ConstrainFloat(GyroGetData().x, -100, 100);
    up_optflow_manager.Gyro_y = ConstrainFloat(GyroGetData().y, -100, 100);

    //LPF_1_(4, 0.02f, up_optflow_manager.Gyro_x, up_optflow_manager.Gyro_x_phase);
    //LPF_1_(4, 0.02f, up_optflow_manager.Gyro_y, up_optflow_manager.Gyro_y_phase);

#ifdef OPT_FLOW_PLACE_BACK_LINE
    up_optflow_manager.Velocity_uncoupled_x = (up_optflow_manager.Velocity_x_lpf - up_optflow_manager.Gyro_x_phase);
    up_optflow_manager.Velocity_uncoupled_y = (up_optflow_manager.Velocity_y_lpf + up_optflow_manager.Gyro_y_phase);

#else
    //陀螺仪解耦合，单位是像素Pix单位时间内像素的位移
    up_optflow_manager.Velocity_uncoupled_x[0] = (up_optflow_manager.Velocity_x_lpf -
                                                  up_optflow_manager.Gyro_x_phase);
    up_optflow_manager.Velocity_uncoupled_y[0] = (up_optflow_manager.Velocity_y_lpf -
                                                  up_optflow_manager.Gyro_y_phase);

#endif
    //光流速度限幅
    up_optflow_manager.Velocity_uncoupled_x[0] = ConstrainFloat(up_optflow_manager.Velocity_uncoupled_x[0], -200.0f,
                                                                200.0f);
    up_optflow_manager.Velocity_uncoupled_y[0] = ConstrainFloat(up_optflow_manager.Velocity_uncoupled_y[0], -200.0f,
                                                                200.0f);

    //LPF
    up_optflow_manager.Velocity_uncoupled_x[1] +=
            0.3f * (up_optflow_manager.Velocity_uncoupled_x[0] - up_optflow_manager.Velocity_uncoupled_x[1]);
    up_optflow_manager.Velocity_uncoupled_y[1] +=
            0.3f * (up_optflow_manager.Velocity_uncoupled_y[0] - up_optflow_manager.Velocity_uncoupled_y[1]);

    //单位时间内地面位移(cm) = 单位时间内像素的位移 * 高度 (unit: cm)
    up_optflow_manager.pos_x = up_optflow_manager.Velocity_uncoupled_x[1] * up_optflow_manager.Ref_height;
    up_optflow_manager.pos_y = up_optflow_manager.Velocity_uncoupled_y[1] * up_optflow_manager.Ref_height;

    //计算地面速度
    up_optflow_manager.Ref_gnd_vel_x = up_optflow_manager.pos_x / 0.02f;
    up_optflow_manager.Ref_gnd_vel_y = up_optflow_manager.pos_y / 0.02f;

    //累计位移
    up_optflow_manager.Gnd_Position_x += up_optflow_manager.Ref_gnd_vel_x * 0.02f;
    up_optflow_manager.Gnd_Position_y += up_optflow_manager.Ref_gnd_vel_y * 0.02f;

    up_optflow_manager.update_flag = true;

    if (pos_reset_cnt < 200) {
        OptflowDataClear();
    }
}

void OptflowDataClear(void) {

}