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

#ifndef MJPC_TASKS_YP_STAND_H_
#define MJPC_TASKS_YP_STAND_H_

#include <memory>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"


namespace mjpc {

double GNLSim(double inputValue, double noiseStdDev, double cutoff);

class YP : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public BaseResidualFn {
   public:
    explicit ResidualFn(const YP* task, int current_mode = 0)
        : mjpc::BaseResidualFn(task) {}
    ResidualFn(const ResidualFn&) = default;
    // ------- Residuals for OP3 task ------------
    //     Residual(0): height - feet height
    //     Residual(1): balance
    //     Residual(2): center of mass xy velocity
    //     Residual(3): ctrl - ctrl_nominal
    //     Residual(4): upright
    //     Residual(5): joint velocity
    // -------------------------------------------
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;

   private:
    friend class YP;

    // modes
    enum YPMode {
      kModeStand = 0,
      kModeWalk,
      kModeJump,
    };
    enum YPFoot {
      kFootLF  = 0,
      kFootLB,
      kFootRF,
      kFootRB,
      kNumFoot
    };
    enum YPGait {
      kGaitStand = 0,
      kGaitWalk,
      kGaitTrot,
      kGaitJump,
      kNumGait
    };
    //  ============  constants  ============
    constexpr static YPFoot kFootAll[kNumFoot] = {kFootLF, kFootLB,
                                                  kFootRF, kFootRB};
    constexpr static YPGait kGaitAll[kNumGait] = {kGaitStand, kGaitWalk,
                                                  kGaitTrot, kGaitJump};
    constexpr static double kGaitPhase[kNumGait][kNumFoot] =
    {
    // LF     LB     RF     RB
      {0,     0,     0,     0   },   // stand
      {0,     0,     0.5,   0.5 },   // walk
      {0,     0,     0.5,   0.5 },   // trot
      {0.5,   0.5,   0.5,   0.5},   // Jump
    };

    constexpr static double kGaitParam[kNumGait][7] =
    {
    // duty ratio  cadence  amplitude  balance   upright   height    goal_pos
    // unitless    Hz       meter      unitless  unitless  unitless  unitless
      {1,          1,       0.03,      0,        1,        1,        0},      // stand
      {0.6,        1,       0.03,      0,        1,        1,        23},     // walk
      {0.4,        1.55,    0.045,     0.2,      1,        1,        15},     // trot
      {0.5,        1,       0.05,      0.03,     0.5,      0.2,      0},      // Jump
    };

    constexpr static double kGaitAuto[kNumGait] =
    {
      0,     // stand
      0.04,  // walk
      0.04,  // trot
      0.6,   // Jump
    };

    // automatic gait switching: time constant for com speed filter
    constexpr static double kAutoGaitFilter = 0.2;    // second

    // automatic gait switching: minimum time between switches
    constexpr static double kAutoGaitMinTime = 1;     // second

    // target torso height over feet when quadrupedal
    constexpr static double kHeightQuadruped = 0.25;  // meter

    // target torso height over feet when bipedal
    constexpr static double kHeightBiped = 0.6;       // meter

    // radius of foot geoms
    constexpr static double kFootRadius = 0.02;       // meter

    // below this target yaw velocity, walk straight
    constexpr static double kMinAngvel = 0.01;        // radian/second

    // posture gain factors for abduction, hip, knee
    constexpr static double kJointPostureGain[3] = {2, 1, 1};  // unitless

    // flip: crouching height, from which leap is initiated
    constexpr static double kCrouchHeight = 0.15;     // meter

    // flip: leap height, beginning of flight phase
    constexpr static double kLeapHeight = 0.5;        // meter

    // flip: maximum height of flight phase
    constexpr static double kMaxHeight = 0.8;         // meter

    //  ============  methods  ============
    // return internal phase clock
    double GetPhase(double time) const;

    // return current gait
    YPGait GetGait() const;

    // compute average foot position, depending on mode
    void AverageFootPos(double avg_foot_pos[3],
                        double* foot_pos[kNumFoot]) const;

    // return normalized target step height
    double StepHeight(double time, double footphase, double duty_ratio) const;

    // compute target step height for all feet
    void FootStep(double step[kNumFoot], double time, YPGait gait) const;

    // walk horizontal position given time
    void Walk(double pos[2], double time) const;

    // height during flip
    double FlipHeight(double time) const;

    // orientation during flip
    void FlipQuat(double quat[4], double time) const;

     //  ============  task state variables, managed by Transition  ============
    YPMode current_mode_       = kModeStand;
    double last_transition_time_ = -1;

    // common mode states
    double mode_start_time_  = 0;
    double position_[3]       = {0};

    // walk states
    double heading_[2]        = {0};
    double speed_             = 0;
    double angvel_            = 0;

    // backflip states
    double ground_            = 0;
    double orientation_[4]    = {0};
    double save_gait_switch_  = 0;
    std::vector<double> save_weight_;

    // gait-related states
    double current_gait_      = kModeStand;
    double phase_start_       = 0;
    double phase_start_time_  = 0;
    double phase_velocity_    = 0;
    double com_vel_[2]        = {0};
    double gait_switch_time_  = 0;

    // ============ constants,m computed in Reset()  ========== // 
    int gait_param_id_        = -1;
    int foot_geom_id_[kNumFoot] = {7, 6, 4, 3};
    int cadence_param_id_     = 4;
    int amplitude_param_id_   = 5;
    int duty_param_id_        = 6;

  };

  YP() : residual_(this) {}
  void TransitionLocked(mjModel* model, mjData* data) override;

  // call base-class Reset, save task-related ids
  void ResetLocked(const mjModel* model) override;

  // default height goals
  constexpr static double kModeHeight[3] = {0.48, 0.42, 0.57};
  constexpr static double kModeGoal[3] = {0, 15, 0};
  constexpr static double KModeGait[3] = {0, 710.0, 0};

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(residual_);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  friend class ResidualFn;
  ResidualFn residual_;
};

}  // namespace mjpc

#endif  // MJPC_TASKS_YP_STAND_H_
