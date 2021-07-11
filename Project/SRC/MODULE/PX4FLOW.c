#include "PX4FLOW.h"
#include "drv_i2c.h"

px4flow_t frame;
px4flow_integral_t integral_frame;

uint8_t raw[32];

bool PX4FLOW_Detect() {

}

void PX4FLOW_Init() {

}

void PX4FLOW_Update() {
    I2C_Multi_Read(&PX4FLOW_I2C, PX4FLOW_ADDRESS, PX4FLOW_DATA_REG, raw, PX4FLOW_FLAME_LEN);
    frame.frame_count = raw[0] << 8 | raw[1];
    frame.pixel_flow_x_sum = raw[2] << 8 | raw[3];
    frame.pixel_flow_y_sum = raw[4] << 8 | raw[5];
    frame.flow_comp_m_x = raw[6] << 8 | raw[7];
    frame.flow_comp_m_y = raw[8] << 8 | raw[9];
    frame.quality = raw[10] << 8 | raw[11];
    frame.gyro_x_rate = raw[12] << 8 | raw[13];
    frame.gyro_y_rate = raw[14] << 8 | raw[15];
    frame.gyro_z_rate = raw[16] << 8 | raw[17];
    frame.gyro_range = raw[18];
    frame.sonar_timestamp = raw[19];
    frame.ground_distance = raw[20] << 8 | raw[21];
}

void PX4FLOW_Integral_Update() {
    I2C_Multi_Read(&PX4FLOW_I2C, PX4FLOW_ADDRESS, PX4FLOW_INTEGRAL_DATA_REG, raw, PX4FLOW_INTEGRAL_FRAME_LEN);
    integral_frame.frame_count_since_last_readout = raw[0] << 8 | raw[1];
    integral_frame.pixel_flow_x_integral = raw[2] << 8 | raw[3];
    integral_frame.pixel_flow_y_integral = raw[4] << 8 | raw[5];
    integral_frame.gyro_x_rate_integral = raw[6] << 8 | raw[7];
    integral_frame.gyro_y_rate_integral = raw[8] << 8 | raw[9];
    integral_frame.gyro_z_rate_integral = raw[10] << 8 | raw[11];
    integral_frame.integration_timespan = raw[12] << 24 | raw[13] << 16 | raw[14] << 8 | raw[15];
    integral_frame.sonar_timestamp = raw[16] << 24 | raw[17] << 16 | raw[18] << 8 | raw[19];
    integral_frame.ground_distance = raw[20] << 8 | raw[21];
    integral_frame.gyro_temperature = raw[22] << 8 | raw[13];
    integral_frame.quality = raw[24];
}

int16_t PX4FLOW_GetPixelFlowX(void) {
    return frame.pixel_flow_x_sum;
}

int16_t PX4FLOW_GetPixelFlowY(void) {
    return frame.pixel_flow_y_sum;
}

int16_t PX4FLOW_GetPixelVelocityX(void) {
    return frame.flow_comp_m_x;
}

int16_t PX4FLOW_GetPixelVelocityY(void) {
    return frame.flow_comp_m_y;
}

int16_t PX4FLOW_GetQuality(void) {
    return frame.quality;
}

uint8_t PX4FLOW_GetSonarTimestamp(void) {
    return frame.sonar_timestamp;
}

int16_t PX4FLOW_GetGroundDistance(void) {
    return frame.ground_distance;
}

int16_t PX4FLOW_GetPixelFlowX_Integral(void) {
    return integral_frame.pixel_flow_x_integral;
}

int16_t PX4FLOW_GetPixelFlowY_Integral(void) {
    return integral_frame.pixel_flow_y_integral;
}

uint32_t PX4FLOW_GetSonarTimestamp_Integral(void) {
    return integral_frame.integration_timespan;
}

int16_t PX4FLOW_GetGroundDistance_Integral(void) {
    return integral_frame.ground_distance;
}