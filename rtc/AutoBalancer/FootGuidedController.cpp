/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "FootGuidedController.h"


void foot_guided_control_base::set_mat()
{
  xi = std::sqrt(g / dz);
  h = 1 + xi * dt;
  h_ = 1 - xi * dt;
  A <<
    1.0, dt,
    xi * xi * dt, 1.0;
  b <<
    0.0,
    -xi * xi * dt;
  Phi <<
    1.0, 1.0 / xi,
    1.0, -1.0 / xi;
  Phi_inv <<
    1.0, 1.0,
    xi, -xi;
  Phi_inv = 2.0 * Phi_inv;
  g_k <<
    0.0,
    -g * dt;
}

double foot_guided_control_base::calc_u(const std::vector<LinearTrajectory<double> >& ref_zmp, const double cur_cp, const bool is_z)
{
  double u = cur_cp - ref_zmp.front().getStart() - ref_zmp.front().getSlope() / xi; // 数式上の j = 0 に相当
  double Tj = 0.0;
  for (size_t j = 0; j < ref_zmp.size() - 1; j++) { // 数式上のindexとは1ずれてる
    Tj += ref_zmp.at(j).getTime();
    u += exp(- xi * Tj) * (ref_zmp.at(j).getGoal() - ref_zmp.at(j+1).getStart() + (ref_zmp.at(j).getSlope() - ref_zmp.at(j+1).getSlope()) / xi);
  }
  if (is_z) u -= dz;
  u = ref_zmp.front().getStart() + 2 * u / (1 - exp(-2 * xi * Tj));

  return u;
}

void foot_guided_control_base::truncate_u()
{
  double acc;
  get_acc(acc);
  if (std::abs(acc) > mu * g) {
    acc = mu * g * sgn(acc);
    u_k = x_k(0) - acc / (xi * xi);
  }
}

// assumed after calc_u
void foot_guided_control_base::calc_x_k(const bool is_z)
{
  x_k = A * x_k + b * u_k;
  if (is_z) x_k += g_k;
}

void foot_guided_control_base::update_control(double& zmp, double& feedforward_zmp, const std::vector<LinearTrajectory<double> >& ref_zmp, const bool is_z)
{
  u_k = calc_u(ref_zmp, (Phi * x_k)(0), is_z);
  act_u_k = calc_u(ref_zmp, (Phi * act_x_k)(0), is_z);

  feedforward_zmp = u_k;
  zmp = act_u_k;
}

void foot_guided_control_base::update_state(double& pos, const double fx, const bool is_z)
{
  calc_x_k(is_z);
  dc_off = - fx / (xi * xi * mass); // 反力
  pos = x_k(0) + dc_off;
}

double foot_guided_control_base::calc_u_jump(const double landing_pos, const double takeoff_height, const double flight_time, const double cur_cog, const double cur_cogvel, const double ref_zmp, const double remain_time, const bool is_z)
{
  double u;
  if (is_z) { // vertical control
    const double takeoff_z_vel = (landing_pos - takeoff_height) / flight_time + g * flight_time / 2.0;
    const double z_d = takeoff_height - cur_cog + (std::exp(-xi * remain_time) * takeoff_z_vel - cur_cogvel) / xi;
    const double z_c = takeoff_height - cur_cog - (std::exp(xi * remain_time) * takeoff_z_vel - cur_cogvel) / xi;
    const double t_d = (std::exp(2 * xi * remain_time) - 1) / (2 * xi);
    const double t_c = (std::exp(-2 * xi * remain_time) - 1) / (2 * xi);
    u = ref_zmp - ((remain_time - t_d) * z_d - (remain_time + t_c) * z_c) / (xi * (remain_time * remain_time + t_c * t_d));
  } else { // horizontal control
    const double r_f = xi * flight_time / 2.0;
    const double x_dp = cur_cog + cur_cogvel / xi - ref_zmp;
    const double x_cp = cur_cog - cur_cogvel / xi - ref_zmp;
    const double x_sp = landing_pos - ref_zmp;
    u = ref_zmp
      -2 * (x_sp - std::exp(xi * remain_time) * x_dp - r_f * (std::exp(xi * remain_time) * x_dp - std::exp(-xi * remain_time) * x_cp))
      / (std::exp(xi * remain_time) - std::exp(-xi * remain_time) + r_f * (std::exp(xi * remain_time) - std::exp(-xi * remain_time) + 2 * xi * std::exp(-xi * remain_time) * remain_time));
  }
  return u;
}

void foot_guided_control_base::update_control_jump(double& zmp, double& feedforward_zmp, const double ref_zmp, const double landing_pos, const double takeoff_height, const double flight_time, const double remain_time, const bool is_z)
{
  u_k = calc_u_jump(landing_pos, takeoff_height, flight_time, x_k(0), x_k(1), ref_zmp, remain_time, is_z);
  act_u_k = calc_u_jump(landing_pos, takeoff_height, flight_time, act_x_k(0), act_x_k(1), ref_zmp, remain_time, is_z);

  feedforward_zmp = u_k;
  zmp = act_u_k;
}

// void foot_guided_control_base::update(double& zmp, double& pos, const std::size_t N, const double ref_dcm, const double ref_zmp)
// {
//   update_control(zmp, N, ref_dcm, ref_zmp);
//   update_state(pos);
// }
