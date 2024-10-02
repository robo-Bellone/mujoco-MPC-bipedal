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
int j = 0;

int height_index = 0;

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

MotorControl mc("ControlSharedMemory");
// ------- Residuals for YP task ------------
//     Residual(0): height - feet height
//     Residual(1): balance
//     Residual(2): center of mass xy velocity
//     Residual(3): feet collision avoid
//     Residual(4): Leg aside FK cost
//     Residual(5): Gait
//     Residual(6): ctrl - ctrl_nominal
//     Residual(7): upright
//     Residual(8): joint velocity
//     Residual(9): Effort
//     Residual(10): position - goal position + heading * goal position
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

  // double* lf = SensorByName(model, data, "LF_touch");
  // double* lb = SensorByName(model, data, "LB_touch");

  // // ----- Actuators ---- //
  // double* hip_r = SensorByName(model, data, "HIP_R");
  // double* hip_l = SensorByName(model, data, "HIP_L");
  // double* thigh_r = SensorByName(model, data, "THIGH_R");
  // double* thigh_l = SensorByName(model, data, "THIGH_L");
  // double* shin_r = SensorByName(model, data, "SHIN_R");
  // double* shin_l = SensorByName(model, data, "SHIN_L");
  // double* foot_r = SensorByName(model, data, "FOOT_R");
  // double* foot_l = SensorByName(model, data, "FOOT_L");

  // ----- Simulating Comm delay ------ //
  //head_position[2] = processor.addData(head_position[2]);
  //left_foot_position[2] = processor1.addData(left_foot_position[2]);
  //right_foot_position[2] = processor2.addData(right_foot_position[2]);
  
  // ----- Simulating Sense noise ------ //
  //head_position[2] = GNLSim(head_position[2]);
  //left_foot_position[2] = GNLSim(left_foot_position[2]);
  //right_foot_position[2] = GNLSim(right_foot_position[2]);

  // get foot positions
  double* foot_pos[kNumFoot];
  for (YPFoot foot : kFootAll) {
    foot_pos[foot] = data->site_xpos + 3 * foot_geom_id_[foot];
    // std::cout << "Foot " << foot << " position: " 
    //           << foot_pos[foot][0] << ", " 
    //           << foot_pos[foot][1] << ", " 
    //           << foot_pos[foot][2] << ", "
    //           << foot_geom_id_[foot] << std::endl;
  }


  // ----- Height ----- //
  if (current_mode_ != kModeJump) {
    double head_feet_error = head_position[2] - 0.5 * (left_foot_position[2] +
                                                       right_foot_position[2]);
    residual[counter++] = head_feet_error - parameters_[0];
  }else{
    double head_feet_error = head_position[2] - 0.5 * (left_foot_position[2] +
                                                       right_foot_position[2]);
    height_index = counter;
    residual[counter++] = head_feet_error - parameters_[0];
  }

  // ----- Balance: CoM-feet xy error ----- //

  // capture point
  double kFallTime = 0.05;
  double capture_point[3] = {com_position[0], com_position[1], com_position[2]};
  mju_addToScl3(capture_point, com_velocity, kFallTime);

  // average feet xy position
  double fxy_avg[2] = {0.0};
  if (current_mode_ != kModeJump) {
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

  // ----- Is Left leg endpoint crossed to the other side plane? ----- //

  double plane_margin_param = parameters_[2];
  double left_fk_cost = ((mjPI - data->qpos[9])/2 * data->qpos[7]) - plane_margin_param;
  double right_fk_cost = ((mjPI - data->qpos[13])/2 * data->qpos[11]) - plane_margin_param;
  left_fk_cost = std::clamp(left_fk_cost, 0.0, left_fk_cost);
  right_fk_cost = std::clamp(right_fk_cost, 0.0, right_fk_cost);
  residual[counter++] = left_fk_cost + right_fk_cost;

  // Debug!
  // if(i>99 && j==1){
  //   std::cout<< "left: " << left_fk_cost << ", right: " << right_fk_cost << std::endl;
  // }

  // ----- Gait ----- //
  YPGait gait = GetGait();
  double step[kNumFoot];
  //std::cout << foot_pos[kFootLF][2] << std::endl;
  //std::cout << data->time << std::endl;
  FootStep(step, GetPhase(data->time), gait);
  for (YPFoot foot : kFootAll) {
    double query[3] = {foot_pos[foot][0], foot_pos[foot][1], foot_pos[foot][2]};
    double ground_height = Ground(model, data, query);
    ground_height = 0;
    double height_target = ground_height + step[foot];
    double height_difference = foot_pos[foot][2] - height_target;
    // if(foot==kFootLF)std::cout << "value: " << height_target <<
    // " ground height: " << ground_height << 
    // " foot postion: " << foot_pos[foot][2] << 
    // " target_step: " << step[foot] << 
    // " Phase of foot: " << GetPhase(data->time) << std::endl;
    //if (foot == kFootLB) std::cout<< step[kFootLB] << std::endl;
    //if (foot == kFootLF) std::cout<< "LF foot z: " <<foot_pos[foot][2] << std::endl;
    residual[counter++] = step[foot] ? height_difference : 0;
    if(current_mode_ == kModeStand || gait == kGaitStand) step[foot] = 0;

    if(gait == kGaitJump){
      if(foot==kFootRB) residual[height_index] += 0.5 * step[foot];
    }else{
      if(foot==kFootLB) residual[height_index] += 0.5 * step[foot];
      if(foot==kFootRB) residual[height_index] += 0.5 * step[foot];
    }
  }


  // ----- Ctrl difference ----- //
  mju_sub(residual + counter, data->ctrl,
          model->key_qpos + model->nq * mode + 7, model->nu);
  counter += model->nu;

  // Debug!
  //data->qpos[0] = 0;
  //std::cout << data->time << std::endl;

  // ----- Upright ----- //
  double standing = -1.0;
  double z_ref[3] = {0.0, 0.0, 1.0};

  if (current_mode_ != kModeStand) {
    // right foot
    mju_sub3(&residual[counter], foot_right_up, z_ref);
    mju_scl3(&residual[counter], &residual[counter], 0.2 * standing);
    counter += 3;

    // left foot
    mju_sub3(&residual[counter], foot_left_up, z_ref);
    mju_scl3(&residual[counter], &residual[counter], 0.2 * standing);
    counter += 3;

    // torso
    residual[counter++] = torso_up[2] - 1.0; // angle of torso

    // zero remaining residual
    mju_zero(residual + counter, 6);
    counter += 6;
  }else{
    // right foot
    mju_sub3(&residual[counter], foot_right_up, z_ref);
    mju_scl3(&residual[counter], &residual[counter], 0.1 * standing);
    counter += 3;

    // left foot
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

  // ---------- Effort -------- //
  mju_scl(residual + counter, data->actuator_force, 2e-2, model->nu);
  counter += model->nu;

  // ----- Position error ----- //
  double position_error[3];
  double* goal_position = data->mocap_pos;
  mju_sub(position_error, head_position, goal_position, 2);
  goal_position[2] = parameters_[0];
  double position_error_norm = mju_norm3(position_error);

  //if(current_mode_ == kModeStand) position_error_norm = 0;
  residual[counter++] = position_error_norm;
  

  //int site_id = mj_name2id(model, mjOBJ_SITE, "LF_touch");

  //sensor data check 
  // double* LFz = SensorByName(model, data, "LF_touch");
  // // double* LBz = SensorByName(model, d, "LB_touch");
  // // double* RFz = SensorByName(model, d, "RF_touch");
  // // double* RBz = SensorByName(model, d, "RB_touch");
  // std::cout << LFz[0] << std::endl;
  // //start of wtf
  // if (i>99 && j==1) {
  //   j = 0;
  //   std::cout << "lf contact: " << lf[0] << "\n";
  //   std::cout << "lb contact: " << lb[0] << "\n";
  // }
  // //end of wtf
  // sensor dim sanity check
  CheckSensorDim(model, counter);
}

void YP::TransitionLocked(mjModel* model, mjData* d) {
  // check for mode change
  if (residual_.current_mode_ == ResidualFn::kModeJump) {
    // update mode for residual
    // set height goal based on mode (stand, jump)
    parameters[0] = kModeHeight[mode];
  }
  if (residual_.current_mode_ == ResidualFn::kModeStand){
    weight[10] = kModeGoal[mode];
    weight[5] = KModeGait[mode];
  }
  

  double gait_selection = parameters[residual_.gait_param_id_];
  if (gait_selection != residual_.current_gait_) {
    residual_.current_gait_ = gait_selection;
    ResidualFn::YPGait gait = residual_.GetGait();
    //std::cout << gait << std::endl;
    parameters[residual_.duty_param_id_] = ResidualFn::kGaitParam[gait][0];
    parameters[residual_.cadence_param_id_] = ResidualFn::kGaitParam[gait][1]; 
    parameters[residual_.amplitude_param_id_] = ResidualFn::kGaitParam[gait][2];
    // weight[residual_.balance_cost_id_] = ResidualFn::kGaitParam[gait][3];
    // weight[residual_.upright_cost_id_] = ResidualFn::kGaitParam[gait][4];
    // weight[residual_.height_cost_id_] = ResidualFn::kGaitParam[gait][5];
  }

  double phase_velocity = 2 * mjPI * parameters[residual_.cadence_param_id_];
  //std::cout << "phase_vel: " << phase_velocity << std::endl;
  //std::cout << "resi_phase_vel: " << residual_.phase_velocity_ << std::endl;
  if (phase_velocity != residual_.phase_velocity_) {
    residual_.phase_start_ = residual_.GetPhase(d->time);
    residual_.phase_start_time_ = d->time;
    residual_.phase_velocity_ = phase_velocity;
  }
  // std::cout << "Parameter at cadence_param_id_: " << parameters[residual_.cadence_param_id_] << std::endl;
  // std::cout << "Calculated phase_velocity: " << phase_velocity << std::endl;
  // std::cout << "Updated phase_velocity: " << residual_.phase_velocity_ << std::endl;


  // if (mode != residual_.current_mode_ &&
  //     residual_.current_mode_ != ResidualFn::kModeStand) {
  //   // switch into stateful mode only allowed from Quadruped
  //   if (mode == ResidualFn::kModeWalk || mode == ResidualFn::kModeJump) {
  //     mode = ResidualFn::kModeStand;
  //   }
  // }
  
  // 조인트의 위치를 설정합니다.
  i += 1;
  // printf("%f\n", center_x_pos);
  //d->qpos[3] = center_z_pos/180*1;
  //d->qpos[3] = 1.0;
  //d->qpos[4] = center_x_pos/180*1;
  //d->qpos[5] = center_y_pos/180*1;
  
  //d->qpos[7] = 6.28 * (center_x_pos / 360);

  // d->qpos[7] = mc.data->feedback[0].position;  // 라디안 단위 //hip r 2
  // d->qpos[8] = mc.data->feedback[1].position;  // 라디안 단위 //thigh r 3
  // d->qpos[9] = mc.data->feedback[2].position;  // 라디안 단위 //shin r 4
  // d->qpos[10] = mc.data->feedback[3].position;  // 라디안 단위 //foot r 5
  // d->qpos[11] = mc.data->feedback[4].position;  // 라디안 단위 //hip r 6
  // d->qpos[12] = mc.data->feedback[5].position;  // 라디안 단위 //thigh r 7
  // d->qpos[13] = mc.data->feedback[6].position;  // 라디안 단위 //shin r 8 
  // d->qpos[14] = mc.data->feedback[7].position;  // 라디안 단위 //foot r 9

  // for(int i = 0; i < sizeof(mc.data->feedback); i++){
  //   d->qpos[i+7] = mc.data->feedback[i].position;
  // }

  if (i>2) {
    i = 0;
    j = 1;
    // mc.readData();
    // Debug!
    // d->qpos[7] = 9 * d->qpos[7] / 10;  // 라디안 단위 //hip r
    // d->qpos[8] = 9 * d->qpos[8] / 10;  // 라디안 단위 //thigh r
    // d->qpos[9] = 9 * d->qpos[9] / 10;  // 라디안 단위 //shin r
    // d->qpos[10] = 9 * d->qpos[10] / 10;  // 라디안 단위 //foot r
    // d->qpos[11] = 9 * d->qpos[11] / 10;  // 라디안 단위 //hip r
    // d->qpos[12] = 9 * d->qpos[12] / 10;  // 라디안 단위 //thigh r
    // d->qpos[13] = 9 * d->qpos[13] / 10;  // 라디안 단위 //shin r
    // d->qpos[14] = 9 * d->qpos[14] / 10;  // 라디안 단위 //foot r

    // d->qpos[7] = mc.data->feedback[0].position;  // 라디안 단위 //hip r 2
    // d->qpos[8] = mc.data->feedback[1].position;  // 라디안 단위 //thigh r 3
    // d->qpos[9] = mc.data->feedback[2].position;  // 라디안 단위 //shin r 4
    // d->qpos[10] = mc.data->feedback[3].position;  // 라디안 단위 //foot r 5
    // d->qpos[11] = mc.data->feedback[4].position;  // 라디안 단위 //hip r 6
    // d->qpos[12] = mc.data->feedback[5].position;  // 라디안 단위 //thigh r 7
    // d->qpos[13] = mc.data->feedback[6].position;  // 라디안 단위 //shin r 8 
    // d->qpos[14] = mc.data->feedback[7].position;  // 라디안 단위 //foot r 9

    //d->xpos[2] = 1;

    // std::cout << "Array values: ";
    // for (int i = 0; i < sizeof(d->actuator_force); i++) {
    //     std::cout << d->actuator_force[i] << " ";  // Using pointer arithmetic to access and print elements
    //     //d-> actuator_force[i] = 5;
    // }
    // std::cout << std::endl;  // End the line after all elements have been printed
    // std::cout << "Xmat Array values: ";
    // for (int i = 0; i < sizeof(SensorByName(model, d, "head_orientation")); i++) {
    //     std::cout << SensorByName(model, d, "head_orientation")[i] << " ";  // Using pointer arithmetic to access and print elements
    // }
    // std::cout << std::endl;  // End the line after all elements have been printed
  }
  residual_.current_mode_ = static_cast<ResidualFn::YPMode>(mode);
  //residual_.last_transition_time_ = d->time;
}
void YP::ResetLocked(const mjModel* model) {
  residual_.gait_param_id_ = ParameterIndex(model, "select_Gait");
  std::cout << "param_id: " << residual_.gait_param_id_ << std::endl;
  int foot_index = 0;
  for (const char* footname : {"LF", "LB", "RF", "RB"}) {
    int foot_id = mj_name2id(model, mjOBJ_SITE, footname);
    std::cout << "foot_index: " << foot_id << std::endl;
    if (foot_id < 0) mju_error_s("site '%s' not found", footname);
    residual_.foot_geom_id_[foot_index] = foot_id;
    foot_index++;
  }
}

// Implementation of the GetPhase method
double YP::ResidualFn::GetPhase(double time) const {
  // std::cout << phase_start_ << ": " << time << ": "<<phase_start_time_ << ": " << phase_velocity_ << std::endl;
  // return phase_start_ + (time - phase_start_time_) * 2 * mjPI * 1;
  double phase = phase_start_ + (time - phase_start_time_) * phase_velocity_;
  // std::cout << "Calculating phase: " << std::endl;
  // std::cout << "  phase_start_: " << phase_start_ << std::endl;
  // std::cout << "  time: " << time << std::endl;
  // std::cout << "  phase_start_time_: " << phase_start_time_ << std::endl;
  // std::cout << "  phase_velocity_: " << phase_velocity_ << std::endl;
  // std::cout << "  Calculated phase: " << phase << std::endl;
  return phase;
}

// Implementation of the GetGait method
YP::ResidualFn::YPGait YP::ResidualFn::GetGait() const {
  return static_cast<YPGait>(ReinterpretAsInt(current_gait_));
}

// return normalized target step height
double YP::ResidualFn::StepHeight(double time, double footphase,
                                             double duty_ratio) const {
  double angle = fmod(time + mjPI - footphase, 2*mjPI) - mjPI;
  double value = 0;
  if (duty_ratio < 1) {
    angle *= 0.5 / (1 - duty_ratio);
    value = mju_cos(mju_clip(angle, -mjPI/2, mjPI/2));
  }
  return mju_abs(value) < 1e-6 ? 0.0 : value;
}

// Implementation of the FootStep method
void YP::ResidualFn::FootStep(double step[kNumFoot], double time,
                                         YPGait gait) const {
  double amplitude = parameters_[amplitude_param_id_];
  double duty_ratio = parameters_[duty_param_id_];
  //std::cout << time << std::endl;
  for (YPFoot foot : kFootAll) {
    double footphase = 2*mjPI*kGaitPhase[gait][foot];
    step[foot] = amplitude * StepHeight(time, footphase, duty_ratio);
  }
}

}  // namespace mjpc