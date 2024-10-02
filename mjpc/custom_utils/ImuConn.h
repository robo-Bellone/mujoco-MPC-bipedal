#ifndef _5_4IMU_3THREADS_H_
#define _5_4IMU_3THREADS_H_

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <iomanip> // for setw and setprecision

// Forward declarations for functions defined in .cpp file
void processIMUData(const char* portName, float& xpos, float& ypos, float& zpos,
                    float& roll, float& pitch, float& yaw,
                    float& xloc, float& yloc, float& zloc);

void processPressureData();

float localPredict(float center_z_loc, float& prev_center_z_loc, float& diff_z);

float left_ankle_hi(float left_x_pos, const float heel, const float toe, float& l_a_h, bool left_f_status, bool left_b_status);

float right_ankle_hi(float right_x_pos, const float heel, const float toe, float& r_a_h, bool right_f_status, bool right_b_status);

void smoothSensorData(float &roll, float &pitch, float &yaw, float &roll_m, float &pitch_m, float &yaw_m, float step, float dif);

// Global variables (consider encapsulating these in a struct or class)
extern float center_x_pos, center_y_pos, center_z_pos, center_roll, center_pitch, center_yaw, center_x_loc, center_y_loc, center_z_loc;
extern float left_x_pos, left_y_pos, left_z_pos, left_roll, left_pitch, left_yaw, left_x_loc, left_y_loc, left_z_loc;
extern float right_x_pos, right_y_pos, right_z_pos, right_roll, right_pitch, right_yaw, right_x_loc, right_y_loc, right_z_loc;

// extern float roll_m, pitch_m, yaw_m;
extern float x_vel, y_vel, z_vel;

extern float prev_center_z_loc;
extern float diff_z;

extern float l_a_h;
extern float r_a_h;

extern const float heel;
extern const float toe;

extern std::atomic<bool> exitFlag;

// Pressure sensor data
extern int left_f, left_b, right_f, right_b;
extern bool left_f_status, left_b_status, right_f_status, right_b_status;
extern const int THRESHOLD_1, THRESHOLD_2, THRESHOLD_3, THRESHOLD_4;

extern int imu_counter;
extern int force_counter;

#endif // _5_4IMU_3THREADS_H_
