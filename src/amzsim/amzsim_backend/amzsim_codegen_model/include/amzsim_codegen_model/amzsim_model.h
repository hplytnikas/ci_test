#pragma once

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>

#include "vcu_codegen_dufour.h"
#include "vehicle_codegen_dufour.h"

#include "amzsim_msgs/msg/car_info.hpp"
#include "amzsim_msgs/msg/res_state.hpp"
#include "amzsim_msgs/msg/state.hpp"
#include "autonomous_msgs/msg/double_stamped.hpp"
#include "vcu_msgs/msg/car_command.hpp"
#include "vcu_msgs/msg/res_state.hpp"
#include "vcu_msgs/msg/velocity_estimation.hpp"

using std::placeholders::_1;
const float DELTA = 0.5;
const int L = 1;

// Struct storing vehicle state
struct State {
  // Position
  double x;
  double y;
  double yaw;
  // Velocity
  double v_x;
  double v_y;
  double r;
  // Acceleration
  double a_x;
  double a_y;
  double steer;
  // Wheel Speeds
  double omega_fl;
  double omega_fr;
  double omega_rr;
  double omega_rl;
  // Fz
  double FzFL;
  double FzFR;
  double FzRR;
  double FzRL;
  // Fy
  double FyFL;
  double FyFR;
  double FyRR;
  double FyRL;
  // SA
  double sa_fl;
  double sa_fr;
  double sa_rr;
  double sa_rl;
  // Torques
  double TFL;
  double TFR;
  double TRR;
  double TRL;
};

class AmzsimModel {
public:
  explicit AmzsimModel(rclcpp::Node::SharedPtr nh);
  ~AmzsimModel();
  void update(const double dt);
  double get_frequency();
  double get_dt();

private:
  // node handle to create subscribers and publishers
  rclcpp::Node::SharedPtr node_;

  // simulink car model
  // vcu_dynamics_codegen sl_car;
  dufour_vcu_codegen sl_vcu;
  dufour_vehicle_codegen sl_car;

  void integrateDynamics(State &x, const vcu_msgs::msg::CarCommand &u, vcu_msgs::msg::ResState &res_go,
                         const double dt);
  void onCmd(const vcu_msgs::msg::CarCommand::SharedPtr msg);
  void onRes(const vcu_msgs::msg::ResState::SharedPtr msg);
  void onRes_sim(const amzsim_msgs::msg::ResState::SharedPtr msg);
  void load_sim_config();

  rclcpp::Subscription<vcu_msgs::msg::ResState>::SharedPtr sub_res_;
  rclcpp::Subscription<amzsim_msgs::msg::ResState>::SharedPtr sub_res_sim_;
  rclcpp::Subscription<vcu_msgs::msg::CarCommand>::SharedPtr sub_cmd_;

  rclcpp::Publisher<amzsim_msgs::msg::State>::SharedPtr pub_amzsim;
  rclcpp::Publisher<vcu_msgs::msg::VelocityEstimation>::SharedPtr pub_ve_gt;
  rclcpp::Publisher<autonomous_msgs::msg::DoubleStamped>::SharedPtr pub_steering_gt;

  amzsim_msgs::msg::CarInfo car_info_;
  amzsim_msgs::msg::ResState res_state_;

  State state_;
  // vcu_dynamics_codegen::ExtU_vcu_dynamics_codegen_T model_inputs_;
  // vcu_dynamics_codegen::ExtY_vcu_dynamics_codegen_T model_outputs_;
  dufour_vcu_codegen::ExtU_vcu_codegen_T model_inputs_vcu;
  dufour_vcu_codegen::ExtY_vcu_codegen_T model_outputs_vcu;
  dufour_vehicle_codegen::ExtU_vehicle_codegen_T model_inputs_vehicle;
  dufour_vehicle_codegen::ExtY_vehicle_codegen_T model_outputs_vehicle;

  vcu_msgs::msg::CarCommand input_;
  vcu_msgs::msg::ResState res_;

  rclcpp::Clock clock;

  double time_last_cmd_;
  double total_sim_time;
  int64_t last_step;

  double frequency_;
  double dt_;
  double tv_ff;
  double tv_exp;
  double tv_p;
  double tv_i;
  double tv_d;
  double ax_m;
  double ax_q;
  double ax_p;
  double ax_i;
  double ax_d;
  double pge;
  bool roll;
};
