// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/yp/stand.h"
#include "mjpc/tasks/yp/SensorDataProcessor.h"

#include "mjpc/custom_utils/MotorControl.h"
#include "mjpc/custom_utils/ImuConn.h"

#include <string>
#include <iostream>
#include <random>
#include <chrono>
#include <algorithm>
#include <vector>
#include <typeinfo>

#include <thread>
#include <pthread.h>

#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"

std::chrono::milliseconds timeDiff(1);
bool __flag = 1;

namespace mjpc {
std::string YP::XmlPath() const { return GetModelPath("yp/YP_standuptask.xml"); }
std::string YP::Name() const { return "YP"; }

int dataSteps = 10;
int i = 0;

std::thread center_thread(processIMUData, "/dev/ttyUSB0", std::ref(center_x_pos), std::ref(center_y_pos), std::ref(center_z_pos),
                    std::ref(center_roll), std::ref(center_pitch), std::ref(center_yaw),
                    std::ref(center_x_loc), std::ref(center_y_loc), std::ref(center_z_loc));
std::thread left_thread(processIMUData, "/dev/ttyUSB1", std::ref(left_x_pos), std::ref(left_y_pos), std::ref(left_z_pos),
                    std::ref(left_roll), std::ref(left_pitch), std::ref(left_yaw),
                    std::ref(left_x_loc), std::ref(left_y_loc), std::ref(left_z_loc));
std::thread right_thread(processIMUData, "/dev/ttyUSB2", std::ref(right_x_pos), std::ref(right_y_pos), std::ref(right_z_pos),
                    std::ref(right_roll), std::ref(right_pitch), std::ref(right_yaw),
                    std::ref(right_x_loc), std::ref(right_y_loc), std::ref(right_z_loc));


int joinThreads() {
    // Ensure threads are joinable before joining
    if (center_thread.joinable()) center_thread.join();
    if (left_thread.joinable()) left_thread.join();
    if (right_thread.joinable()) right_thread.join();
    return 0;
}

int joinThreads();


SensorDataProcessor::SensorDataProcessor(int targetDelayms)
    : targetDelay(std::chrono::milliseconds(targetDelayms)) {}

double SensorDataProcessor::addData(double sensorData) {
  dataSteps = 1000;
  // if (dataSteps == 0) {
  //   printf("deal\n");
  //   dataSteps = 1;
  //   return sensorData;
  // }
  auto now = std::chrono::steady_clock::now();
  dataQueue.push(sensorData);
  timeQueue.push(now);
  printf("Queue size: %zu\n", dataQueue.size());
  if (static_cast<int>(dataQueue.size()) > dataSteps) {
    double delayedData = dataQueue.front();
    auto delayedTime = timeQueue.front();

    std::printf("Delayed data: %f\n", delayedData);
    dataQueue.pop();
    timeQueue.pop();

    timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(now - delayedTime);
    std::printf("Delayed time of sensor data %s\n", std::to_string(timeDiff.count()).c_str());
    //std::printf("deal");

    // if(timeDiff > targetDelay) {
    //   dataSteps = dataSteps - 1;
    // }
    // else if(timeDiff < targetDelay) {
    //   dataSteps = dataSteps + 1;
    // }
    return delayedData;
  }
  return 0.0;  // Default 값 반환
}


double noiseStdDev = 0.04;
double cutoff = 1;

double GNLSim(double inputValue, double noiseStdDev = 0.01, double cutoff = 0.05) {
  static std::random_device rd; 
  static std::mt19937 gen(rd()); // 난수 생성 알고리즘 (Mersenne Twister)
  std::normal_distribution<> dist(0, noiseStdDev); // 평균 0, 표준편차 noiseStdDev인 정규분포
  double noise = dist(gen); // 노이즈 생성

  // 노이즈가 cutoff 범위를 벗어나면, cutoff 값으로 제한
  if (noise > cutoff) {
      noise = cutoff;
  } else if (noise < -cutoff) {
      noise = -cutoff;
  }
  return inputValue + noise; // 입력값에 노이즈 추가
}

SensorDataProcessor processor(1000);
SensorDataProcessor processor1(1000);
SensorDataProcessor processor2(1000);

//MotorControl mc("ControlSharedMemory");
// ------- Residuals for YP task ------------
//     Residual(0): height - feet height
//     Residual(1): balance
//     Residual(2): center of mass xy velocity
//     Residual(3): feet collision avoid
//     Residual(4): ctrl - ctrl_nominal
//     Residual(5): upright
//     Residual(6): joint velocity
//     Residual(7): position - goal position
// -------------------------------------------
void YP::ResidualFn::Residual(const mjModel* model, const mjData* data,
                               double* residual) const {
  
  // start counter
  int counter = 0;
  
  // get mode
  int mode = current_mode_;

  //ABSL_FLAG(int, std::to_integer(timeDiff.count()), 10, "Delayed time of sensor data");

  // ----- sensors ------ //
  double* head_position = SensorByName(model, data, "head_position");
  double* left_foot_position = SensorByName(model, data, "left_foot_position");
  double* right_foot_position = SensorByName(model, data, "right_foot_position");
  double* torso_up = SensorByName(model, data, "torso_up");
  double* foot_right_up = SensorByName(model, data, "foot_right_up");
  double* foot_left_up = SensorByName(model, data, "foot_left_up");
  double* com_position = SensorByName(model, data, "body_subtreecom");
  double* com_velocity = SensorByName(model, data, "body_subtreelinvel");

  // // ----- Actuators ---- //
  // double* hip_r = SensorByName(model, data, "HIP_R");
  // double* hip_l = SensorByName(model, data, "HIP_L");
  // double* thigh_r = SensorByName(model, data, "THIGH_R");
  // double* thigh_l = SensorByName(model, data, "THIGH_L");
  // double* shin_r = SensorByName(model, data, "SHIN_R");
  // double* shin_l = SensorByName(model, data, "SHIN_L");
  // double* foot_r = SensorByName(model, data, "FOOT_R");
  // double* foot_l = SensorByName(model, data, "FOOT_L");
  
  // int hip_r_joint_id = mj_name2id(model, mjOBJ_JOINT, "HIP_R");
  //   // 조인트의 위치를 설정합니다.
  // data->qpos[hip_r_joint_id] = 1.57;  // 라디안 단위

  // // 조인트의 속도를 설정합니다.
  // data->qvel[hip_r_joint_id] = 0;  // 라디안/초

  //std::printf("%f  %f  %f  %f  %lu  \n", hip_r[0], hip_r[1], hip_r[2], hip_r[3], sizeof(hip_r));
  // hip_r[0] = 0;
  // hip_r[1] = 0;
  // hip_r[2]
  // hip_l[2] = 0;
  // thigh_r[2] = 0;
  // thigh_l[2] = 1.57;
  // shin_r[2] = 0;
  // shin_l[2] = -0.3;
  // foot_l[2] = 0;
  // foot_r[2] = 0;

  // ----- Simulating Comm delay ------ //
  //head_position[2] = processor.addData(head_position[2]);
  //left_foot_position[2] = processor1.addData(left_foot_position[2]);
  //right_foot_position[2] = processor2.addData(right_foot_position[2]);
  
  // ----- Simulating Sense noise ------ //
  //head_position[2] = GNLSim(head_position[2]);
  //left_foot_position[2] = GNLSim(left_foot_position[2]);
  //right_foot_position[2] = GNLSim(right_foot_position[2]);

  // ----- Height ----- //
  if (mode == kModeStand) {
    double head_feet_error = head_position[2] - 0.5 * (left_foot_position[2] +
                                                       right_foot_position[2]);
    residual[counter++] = head_feet_error - parameters_[0];
  }

  // ----- Balance: CoM-feet xy error ----- //

  // capture point
  double kFallTime = 0.05;
  double capture_point[3] = {com_position[0], com_position[1], com_position[2]};
  mju_addToScl3(capture_point, com_velocity, kFallTime);

  // average feet xy position
  double fxy_avg[2] = {0.0};
  if (mode == kModeStand) {
    mju_addTo(fxy_avg, left_foot_position, 2);
    mju_addTo(fxy_avg, right_foot_position, 2);
  }
  mju_scl(fxy_avg, fxy_avg, 0.5, 2);
  mju_subFrom(fxy_avg, capture_point, 2);
  double com_feet_distance = mju_norm(fxy_avg, 2);
  residual[counter++] = com_feet_distance;

  // ----- COM xy velocity should be 0 ----- //
  mju_copy(&residual[counter], com_velocity, 2);
  counter += 2;

  // ----- Is left foot and right foot too close? ----- //
  double foot_distance = 0.0;
  double foot_distance_norm[3] = {0.0, 0.0};

  mju_addTo(foot_distance_norm, left_foot_position, 3);
  mju_subFrom(foot_distance_norm, right_foot_position, 3);
  double foot_distance_xy[2] = {foot_distance_norm[0], foot_distance_norm[1]};
  foot_distance = mju_norm(foot_distance_xy, 2);
  if(parameters_[1] - foot_distance < 0.0) {
    residual[counter++] = 0.0;
  }else{
    residual[counter++] = parameters_[1] - foot_distance;
  }
  
  //std.::printf("%f\n", data->ctrl[0]);
  //data->qpos[0] = 0;
  // ----- Ctrl difference ----- //
  mju_sub(residual + counter, data->ctrl,
          model->key_qpos + model->nq * mode + 7, model->nu);
  counter += model->nu;

  //data->qpos[0] = 0;

  // ----- Upright ----- //
  double standing = -1.0;
  double z_ref[3] = {0.0, 0.0, 1.0};

  if (mode == kModeStand) {
    // right foot
    mju_sub3(&residual[counter], foot_right_up, z_ref);
    mju_scl3(&residual[counter], &residual[counter], 0.1 * standing);
    counter += 3;

    mju_sub3(&residual[counter], foot_left_up, z_ref);
    mju_scl3(&residual[counter], &residual[counter], 0.1 * standing);
    counter += 3;

    // torso
    residual[counter++] = torso_up[2] - 1.0; // angle of torso

    // zero remaining residual
    mju_zero(residual + counter, 6);
    counter += 6;
  }

  // ----- Joint velocity ----- //
  mju_copy(residual + counter, data->qvel + 6, model->nv - 6);
  counter += model->nv - 6;

  // ----- Position error ----- //
  double position_error[3];
  double* goal_position = data->mocap_pos;
  mju_sub3(position_error, head_position, goal_position);
  goal_position[2] = parameters_[0];
  double position_error_norm = mju_norm3(position_error);
  residual[counter++] = position_error_norm;

  //int site_id = mj_name2id(model, mjOBJ_SITE, "LF_touch");

  //sensor data check 
  //double LFz = data->sensordata[site_id];
  // double* LBz = SensorByName(model, d, "LB_touch");
  // double* RFz = SensorByName(model, d, "RF_touch");
  // double* RBz = SensorByName(model, d, "RB_touch");
  //std::cout << LFz << std::endl;

  // sensor dim sanity check
  CheckSensorDim(model, counter);
}

void YP::TransitionLocked(mjModel* model, mjData* d) {

  // check for mode change
  if (residual_.current_mode_ != mode) {
    // update mode for residual
    residual_.current_mode_ = mode;

    // set height goal based on mode (stand, jump)
    parameters[0] = kModeHeight[mode];
  }
  // 조인트의 위치를 설정합니다.
  i += 1;
  //d->qpos[7] = 6.28 * (center_x_pos / 360);

  if (i>5000) {
    i = 0;
    // d->qpos[7] = 9 * d->qpos[7] / 10;  // 라디안 단위 //hip r
    // d->qpos[8] = 9 * d->qpos[8] / 10;  // 라디안 단위 //thigh r
    // d->qpos[9] = 9 * d->qpos[9] / 10;  // 라디안 단위 //shin r
    // d->qpos[10] = 9 * d->qpos[10] / 10;  // 라디안 단위 //foot r
    // d->qpos[11] = 9 * d->qpos[11] / 10;  // 라디안 단위 //hip r
    // d->qpos[12] = 9 * d->qpos[12] / 10;  // 라디안 단위 //thigh r
    // d->qpos[13] = 9 * d->qpos[13] / 10;  // 라디안 단위 //shin r
    // d->qpos[14] = 9 * d->qpos[14] / 10;  // 라디안 단위 //foot r

    //d->xpos[2] = 1;

    std::cout << "Array values: ";
    for (int i = 0; i < sizeof(d->actuator_force); i++) {
        std::cout << d->actuator_force[i] << " ";  // Using pointer arithmetic to access and print elements
        d-> actuator_force[i] = 5;
    }
    std::cout << std::endl;  // End the line after all elements have been printed
  }
}

}  // namespace mjpc
