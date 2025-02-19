/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2023-2024 Authors:
 *   - Romir Damle <rdamle@ethz.ch>
 *   - Emil Fahrig <efahrig@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "amzsim_codegen_model/amzsim_model.h"

AmzsimModel::AmzsimModel(rclcpp::Node::SharedPtr nh) : node_(nh), sl_car() {
  // Initialize code gen model
  sl_vcu.initialize();
  sl_car.initialize();

  // initialize all subscribers
  sub_res_ = node_->create_subscription<vcu_msgs::msg::ResState>("/vcu_msgs/res_state", 10,
                                                                 std::bind(&AmzsimModel::onRes, this, _1));
  sub_res_sim_ = node_->create_subscription<amzsim_msgs::msg::ResState>("/amzsim_msgs/res_state", 10,
                                                                        std::bind(&AmzsimModel::onRes_sim, this, _1));
  sub_cmd_ = node_->create_subscription<vcu_msgs::msg::CarCommand>("/control/car_command", 10,
                                                                   std::bind(&AmzsimModel::onCmd, this, _1));

  // initialize all publishers
  pub_amzsim = node_->create_publisher<amzsim_msgs::msg::State>("/amzsim/car/state", 1);
  pub_ve_gt = node_->create_publisher<vcu_msgs::msg::VelocityEstimation>("/vcu_msgs/velocity_estimation_gt", 5);
  pub_steering_gt = node_->create_publisher<autonomous_msgs::msg::DoubleStamped>("/vcu_msgs/steering_gt", 5);

  // Node parameters
  node_->declare_parameter("tv_ff", 0.6);
  node_->declare_parameter("tv_exp", 2.45);
  node_->declare_parameter("tv_p", 325.0);
  node_->declare_parameter("tv_i", 0.0);
  node_->declare_parameter("tv_d", 0.0);
  node_->declare_parameter("ax_m", 176.0);
  node_->declare_parameter("ax_q", 100.0);
  node_->declare_parameter("ax_p", 600.0);
  node_->declare_parameter("ax_i", 100.0);
  node_->declare_parameter("ax_d", 0.0);
  node_->declare_parameter("pge", 0.0);
  node_->declare_parameter("roll", false);

  if (!node_->get_parameter("tv_ff", tv_ff)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("tv_exp", tv_exp)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("tv_p", tv_p)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("tv_i", tv_i)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("tv_d", tv_d)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("ax_m", ax_m)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("ax_q", ax_q)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("ax_p", ax_p)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("ax_i", ax_i)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("ax_d", ax_d)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("pge", pge)) {
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("roll", roll)) {
    rclcpp::shutdown();
  }

  // Set states to 0
  state_.x = 0;
  state_.y = 0;
  state_.yaw = 0;

  state_.v_x = 0;
  state_.v_y = 0;
  state_.r = 0;

  state_.a_x = 0;
  state_.a_y = 0;
  state_.steer = 0;

  state_.omega_fl = 0;
  state_.omega_fr = 0;
  state_.omega_rr = 0;
  state_.omega_rl = 0;

  state_.FzFL = 0;
  state_.FzRL = 0;
  state_.FzRR = 0;
  state_.FzRL = 0;

  state_.FyFL = 0;
  state_.FyRL = 0;
  state_.FyRR = 0;
  state_.FyRL = 0;

  state_.sa_fl = 0;
  state_.sa_fr = 0;
  state_.sa_rr = 0;
  state_.sa_rl = 0;

  state_.TFL = 0;
  state_.TFR = 0;
  state_.TRR = 0;
  state_.TRL = 0;

  node_->declare_parameter("frequency", 0.0);
  node_->declare_parameter("dt", 0.0);

  load_sim_config();

  time_last_cmd_ = 0.0;
  total_sim_time = 0.0;
  last_step = 0;
}

// Loads the configuration of the faked perception
void AmzsimModel::load_sim_config() {
  if (!node_->get_parameter("frequency", frequency_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find frequency parameter in load_sim_config()!");
    rclcpp::shutdown();
  }
  if (!node_->get_parameter("dt", dt_)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find dt parameter in load_sim_config()!");
    rclcpp::shutdown();
  }
}

// Return the private attribute frequency
double AmzsimModel::get_frequency() { return frequency_; }

// Return the private attribute dt
double AmzsimModel::get_dt() { return dt_; }

void AmzsimModel::update(const double dt) {
  if (!car_info_.torque_ok || clock.now().seconds() - time_last_cmd_ > 1.0) {
    if (!car_info_.torque_ok) {
      total_sim_time += 0;
      // model_inputs_vehicle.Params_res = 0;
    }

    else if (clock.now().seconds() - time_last_cmd_ > 1.0) {
      total_sim_time += 0;
      // model_inputs_vehicle.Params_res = 0;
    }
  } else {
    // model_inputs_vehicle.Params_res = 1;
  }

  integrateDynamics(state_, input_, res_, dt);

  amzsim_msgs::msg::State msg_amzsim;
  vcu_msgs::msg::VelocityEstimation msg_ve;
  autonomous_msgs::msg::DoubleStamped msg_steering;

  auto t_now = clock.now();

  msg_amzsim.x = state_.x;
  msg_amzsim.y = state_.y;
  msg_amzsim.yaw = state_.yaw;
  msg_amzsim.vx = state_.v_x;
  msg_amzsim.vy = state_.v_y;
  msg_amzsim.dyaw = state_.r;
  msg_amzsim.header.stamp = t_now;

  msg_ve.vel.x = state_.v_x;
  msg_ve.vel.y = state_.v_y;
  msg_ve.vel.theta = state_.r;
  msg_ve.acc.x = state_.a_x;
  msg_ve.acc.y = state_.a_y;
  msg_ve.acc.theta = 0; // Change when added in model
  msg_ve.header.stamp = t_now;

  msg_steering.data = state_.steer;
  msg_steering.header.stamp = t_now;

  pub_steering_gt->publish(msg_steering);
  pub_amzsim->publish(msg_amzsim);
  pub_ve_gt->publish(msg_ve);
}

void AmzsimModel::integrateDynamics(State &x, const vcu_msgs::msg::CarCommand &u, vcu_msgs::msg::ResState &res_go,
                                    const double dt) {
  total_sim_time += dt;

  // Set Model States
  model_inputs_vcu.Params_tv_ff_gain = tv_ff;
  model_inputs_vcu.Params_ax_P = ax_p;
  model_inputs_vcu.Params_ax_I = ax_i;
  model_inputs_vcu.Params_ax_D = ax_d;
  model_inputs_vcu.Params_tv_ff_exp = tv_exp;
  model_inputs_vcu.Params_tv_P = tv_p;
  model_inputs_vcu.Params_tv_I = tv_i;
  model_inputs_vcu.Params_tv_D = tv_d;
  model_inputs_vcu.Params_ax_q = ax_q;
  model_inputs_vcu.Params_ax_m = ax_m;
  model_inputs_vcu.States_delta = x.steer;
  model_inputs_vcu.States_vx = x.v_x;
  model_inputs_vcu.States_vy = x.v_y;
  model_inputs_vcu.States_ax_sensor = x.a_x;
  model_inputs_vcu.States_ay = x.a_y;
  model_inputs_vcu.States_omega_fl = x.omega_fl;
  model_inputs_vcu.States_omega_fr = x.omega_fr;
  model_inputs_vcu.States_omega_rr = x.omega_rr;
  model_inputs_vcu.States_omega_rl = x.omega_rl;
  model_inputs_vcu.States_sa_fl = x.sa_fl;
  model_inputs_vcu.States_sa_fr = x.sa_fr;
  model_inputs_vcu.States_sa_rr = x.sa_rr;
  model_inputs_vcu.States_sa_rl = x.sa_rl;
  model_inputs_vcu.States_FzFL = x.FzFL;
  model_inputs_vcu.States_FzFR = x.FzFR;
  model_inputs_vcu.States_FzRR = x.FzRR;
  model_inputs_vcu.States_FzRL = x.FzRL;
  model_inputs_vcu.States_FyFL = x.FyFL;
  model_inputs_vcu.States_FyFR = x.FyFR;
  model_inputs_vcu.States_FyRR = x.FyRR;
  model_inputs_vcu.States_FyRL = x.FyRL;
  model_inputs_vcu.States_TFL = x.TFL;
  model_inputs_vcu.States_TFR = x.TFR;
  model_inputs_vcu.States_TRR = x.TRR;
  model_inputs_vcu.States_TRL = x.TRL;
  model_inputs_vcu.States_psi_dot = x.r;

  // Set Model Inputs
  model_inputs_vcu.Inputs_cmd_throttle_CB = 0; // Legacy command set to 0
  model_inputs_vcu.Inputs_AS_Mission = 7;      // Can make a ROS input
  model_inputs_vcu.Inputs_a_X_target_0 = u.a_x[0];
  model_inputs_vcu.Inputs_a_X_target_1 = u.a_x[1];
  model_inputs_vcu.Inputs_a_X_target_2 = u.a_x[2];
  model_inputs_vcu.Inputs_Yaw_rate_target_0 = 0; // Yaw rate target can be listened to once MPC starts publishing
  model_inputs_vcu.Inputs_Yaw_rate_target_1 = 0;
  model_inputs_vcu.Inputs_Yaw_rate_target_2 = 0;
  model_inputs_vcu.Inputs_steering_target_0 = u.steering_angle[0];
  model_inputs_vcu.Inputs_steering_target_1 = u.steering_angle[1];
  model_inputs_vcu.Inputs_steering_target_2 = u.steering_angle[2];
  model_inputs_vcu.Inputs_cmd_steering_CB = 0; // Legacy command set to 0
  model_inputs_vcu.Inputs_mpc_llc = 0;         // Don't use mpc llc (flag in vcu)

  sl_vcu.setExternalInputs(&model_inputs_vcu);
  sl_vcu.step();

  model_outputs_vcu = sl_vcu.getExternalOutputs();

  // RCLCPP_INFO(node_->get_logger(), "Torque sent %f", model_outputs_vcu.Outputs_T_M_FL_ref); //Debug

  model_inputs_vehicle.Inputs_T_M_FL_ref = model_outputs_vcu.Outputs_T_M_FL_ref;
  model_inputs_vehicle.Inputs_T_M_FR_ref = model_outputs_vcu.Outputs_T_M_FR_ref;
  model_inputs_vehicle.Inputs_T_M_RL_ref = model_outputs_vcu.Outputs_T_M_RL_ref;
  model_inputs_vehicle.Inputs_T_M_RR_ref = model_outputs_vcu.Outputs_T_M_RR_ref;
  model_inputs_vehicle.Inputs_omega_M_FL_ref = model_outputs_vcu.Outputs_omega_M_FL_ref;
  model_inputs_vehicle.Inputs_omega_M_FR_ref = model_outputs_vcu.Outputs_omega_M_FR_ref;
  model_inputs_vehicle.Inputs_omega_M_RL_ref = model_outputs_vcu.Outputs_omega_M_RL_ref;
  model_inputs_vehicle.Inputs_omega_M_RR_ref = model_outputs_vcu.Outputs_omega_M_RR_ref;
  model_inputs_vehicle.Inputs_omega_M_FL_upperbound = model_outputs_vcu.Outputs_omega_M_FL_upperbound;
  model_inputs_vehicle.Inputs_omega_M_FR_upperbound = model_outputs_vcu.Outputs_omega_M_FR_upperbound;
  model_inputs_vehicle.Inputs_omega_M_RL_upperbound = model_outputs_vcu.Outputs_omega_M_RL_upperbound;
  model_inputs_vehicle.Inputs_omega_M_RR_upperbound = model_outputs_vcu.Outputs_omega_M_RR_upperbound;
  model_inputs_vehicle.Inputs_omega_M_FL_lowerbound = model_outputs_vcu.Outputs_omega_M_FL_lowerbound;
  model_inputs_vehicle.Inputs_omega_M_FR_lowerbound = model_outputs_vcu.Outputs_omega_M_FR_lowerbound;
  model_inputs_vehicle.Inputs_omega_M_RL_lowerbound = model_outputs_vcu.Outputs_omega_M_RL_lowerbound;
  model_inputs_vehicle.Inputs_omega_M_RR_lowerbound = model_outputs_vcu.Outputs_omega_M_RR_lowerbound;
  model_inputs_vehicle.Inputs_steering_target_rad = model_outputs_vcu.Outputs_steering_target_rad;
  model_inputs_vehicle.States_vx = x.v_x;
  model_inputs_vehicle.States_vy = x.v_y;
  model_inputs_vehicle.States_ax = x.a_x;
  model_inputs_vehicle.States_ay = x.a_y;
  model_inputs_vehicle.States_psi_dot = x.r;
  model_inputs_vehicle.States_omega_fl = x.omega_fl;
  model_inputs_vehicle.States_omega_fr = x.omega_fr;
  model_inputs_vehicle.States_omega_rr = x.omega_rl;
  model_inputs_vehicle.States_omega_rl = x.omega_rr;
  model_inputs_vehicle.Params_pge = pge * 0.01; // Pge power comes in percentage
  model_inputs_vehicle.Params_tau_delta = 0.08;
  if (res_go.push_button) {
    model_inputs_vehicle.Params_res = 1;
  } else {
    model_inputs_vehicle.Params_res = 0;
  }
  if (roll == true) {
    model_inputs_vehicle.Params_roll_high = 1;
  } else {
    model_inputs_vehicle.Params_roll_high = 0;
  }

  int64_t target_step = total_sim_time / 0.001;

  sl_car.setExternalInputs(&model_inputs_vehicle);

  for (int i = 1; i < 6; i++) {
    // sl_car.initialize();
    sl_car.update();
    sl_car.output(); // Model runs at 1000 Hz while vcu runs at 200 Hz
    model_outputs_vehicle = sl_car.getExternalOutputs();
    model_inputs_vehicle.States_vx = model_outputs_vehicle.Model_Outputs_vx;
    model_inputs_vehicle.States_vy = model_outputs_vehicle.Model_Outputs_vy;
    model_inputs_vehicle.States_ax = model_outputs_vehicle.Model_Outputs_ax;
    model_inputs_vehicle.States_ay = model_outputs_vehicle.Model_Outputs_ay;
    model_inputs_vehicle.States_psi_dot = model_outputs_vehicle.Model_Outputs_psi_dot;
    model_inputs_vehicle.States_omega_fl = model_outputs_vehicle.Model_Outputs_omega_fl;
    model_inputs_vehicle.States_omega_fr = model_outputs_vehicle.Model_Outputs_omega_fr;
    model_inputs_vehicle.States_omega_rr = model_outputs_vehicle.Model_Outputs_omega_rr;
    model_inputs_vehicle.States_omega_rl = model_outputs_vehicle.Model_Outputs_omega_rl;
    sl_car.setExternalInputs(&model_inputs_vehicle);
  }

  // RCLCPP_INFO(node_->get_logger(), "vx: %f", model_outputs_vehicle.Model_Outputs_vx); //Debug

  model_outputs_vehicle = sl_car.getExternalOutputs();
  x.v_x = model_outputs_vehicle.Model_Outputs_vx;
  x.v_y = model_outputs_vehicle.Model_Outputs_vy;
  x.y = model_outputs_vehicle.Model_Outputs_py;
  x.x = model_outputs_vehicle.Model_Outputs_px;
  x.yaw = model_outputs_vehicle.Model_Outputs_psi;
  x.r = model_outputs_vehicle.Model_Outputs_psi_dot;
  x.a_x = model_outputs_vehicle.Model_Outputs_ax_sensor;
  x.a_y = model_outputs_vehicle.Model_Outputs_ay;
  x.steer = model_outputs_vehicle.Model_Outputs_delta;
  x.omega_fl = model_outputs_vehicle.Model_Outputs_omega_fl;
  x.omega_fr = model_outputs_vehicle.Model_Outputs_omega_fr;
  x.omega_rr = model_outputs_vehicle.Model_Outputs_omega_rr;
  x.omega_rl = model_outputs_vehicle.Model_Outputs_omega_rl;
  x.FzFL = model_outputs_vehicle.Model_Outputs_FzFL;
  x.FzFR = model_outputs_vehicle.Model_Outputs_FzFR;
  x.FzRR = model_outputs_vehicle.Model_Outputs_FzRR;
  x.FzRL = model_outputs_vehicle.Model_Outputs_FzRL;
  x.FyFL = model_outputs_vehicle.Model_Outputs_FyFL;
  x.FyFR = model_outputs_vehicle.Model_Outputs_FyFR;
  x.FyRR = model_outputs_vehicle.Model_Outputs_FyRR;
  x.FyRL = model_outputs_vehicle.Model_Outputs_FyRL;
  x.sa_fl = model_outputs_vehicle.Model_Outputs_sa_fl;
  x.sa_fr = model_outputs_vehicle.Model_Outputs_sa_fr;
  x.sa_rr = model_outputs_vehicle.Model_Outputs_sa_rr;
  x.sa_rl = model_outputs_vehicle.Model_Outputs_sa_rl;
  x.TFL = model_outputs_vehicle.Model_Outputs_TFL;
  x.TFR = model_outputs_vehicle.Model_Outputs_TFR;
  x.TRR = model_outputs_vehicle.Model_Outputs_TRR;
  x.TRL = model_outputs_vehicle.Model_Outputs_TRL;
}

void AmzsimModel::onRes(const vcu_msgs::msg::ResState::SharedPtr msg) { res_ = *msg; }

void AmzsimModel::onRes_sim(const amzsim_msgs::msg::ResState::SharedPtr msg) {
  res_state_ = *msg;
  if (res_state_.push_button) {
    car_info_.torque_ok = static_cast<unsigned char>(true);
    // model_inputs_vehicle.Params_res = 1;
  }
  if (res_state_.emergency) {
    car_info_.torque_ok = static_cast<unsigned char>(false);
    // model_inputs_vehicle.Params_res = 0;
  }
}

void AmzsimModel::onCmd(const vcu_msgs::msg::CarCommand::SharedPtr msg) {
  input_ = *msg;
  time_last_cmd_ = clock.now().seconds();
}

AmzsimModel::~AmzsimModel() {
  sl_car.terminate();
  sl_vcu.terminate();
}
