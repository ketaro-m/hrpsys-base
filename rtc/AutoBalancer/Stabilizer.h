// -*- C++ -*-
/*!
 * @file  Stabilizer.h
 * @brief stabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef STABILIZER_COMPONENT_H
#define STABILIZER_COMPONENT_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AutoBalancerService_impl.h"
#include "TwoDofController.h"
#include "ZMPDistributor.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"
#include "../SequencePlayer/interpolator.h"
#include "hrpsys/idl/RobotHardwareService.hh"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

/**
   \brief sample RT component which has one data input port and one data output port
 */

class Stabilizer
{
private:
  // Robot model
  hrp::BodyPtr m_robot;
  double dt;
  std::string print_str;

public:
  enum cphase {LANDING_PHASE=-1, SWING_PHASE=0, SUPPORT_PHASE=1};
  struct STIKParam {
    std::string target_name; // Name of end link
    std::string ee_name; // Name of ee (e.g., rleg, lleg, ...)
    std::string sensor_name; // Name of force sensor in the limb
    std::string parent_name; // Name of parent ling in the limb
    hrp::Vector3 localp; // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
    hrp::Vector3 localCOPPos; // Position offset of reference COP in end link frame (^{l}p_{cop} = R_l^T (p_{cop} - p_l) - ^{l}p_e)
    hrp::Matrix33 localR; // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)
    // For eefm
    hrp::Vector3 d_foot_pos, ee_d_foot_pos, d_foot_rpy, ee_d_foot_rpy;
    hrp::Vector3 eefm_pos_damping_gain, eefm_pos_time_const_support, eefm_rot_damping_gain, eefm_rot_time_const, eefm_swing_rot_spring_gain, eefm_swing_pos_spring_gain, eefm_swing_rot_time_const, eefm_swing_pos_time_const, eefm_ee_moment_limit;
    double eefm_pos_compensation_limit, eefm_rot_compensation_limit;
    hrp::Vector3 ref_force, ref_moment;
    hrp::dvector6 eefm_ee_forcemoment_distribution_weight;
    double swing_support_gain, support_time;
    // For swing ee modification
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > target_ee_diff_p_filter;
    hrp::Vector3 target_ee_diff_p, d_pos_swing, d_rpy_swing, prev_d_pos_swing, prev_d_rpy_swing;
    double touchoff_remain_time;
    // IK parameter
    double avoid_gain, reference_gain, max_limb_length, limb_length_margin;
    size_t ik_loop_count;
    Eigen::AngleAxisd ref_theta, act_theta, omega;
    // joint servo control parameter
    cphase contact_phase;
    double phase_time;
    hrp::dvector support_pgain,support_dgain,landing_pgain,landing_dgain, swing_pgain, swing_dgain;
  };
  enum cmode {MODE_IDLE, MODE_AIR, MODE_ST, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_AIR} control_mode;
  // members
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  std::vector<hrp::JointPathExPtr> jpe_v;
  coil::Mutex m_mutex;
  unsigned int m_debugLevel;
  hrp::dvector transition_joint_q, qorg, qrefv;
  std::vector<STIKParam> stikp;
  std::map<std::string, size_t> contact_states_index_map;
  std::vector<bool> ref_contact_states, prev_ref_contact_states, act_contact_states, prev_act_contact_states, is_ik_enable, is_feedback_control_enable, is_zmp_calc_enable;
  std::vector<double> toeheel_ratio;
  int transition_count, loop;
  int m_is_falling_counter;
  std::vector<int> m_will_fall_counter;
  int is_air_counter, detection_count_to_air;
  bool is_legged_robot, on_ground, is_emergency, is_seq_interpolating, reset_emergency_flag, eefm_use_force_difference_control, eefm_use_swing_damping, initial_cp_too_large_error, use_limb_stretch_avoidance, use_zmp_truncation;
  bool is_walking, is_estop_while_walking, is_emergency_step, is_single_walking, is_emergency_motion;
  hrp::Vector3 current_root_p, target_root_p, ref_foot_origin_pos, prev_act_foot_origin_pos;
  hrp::Matrix33 current_root_R, target_root_R, act_root_R, prev_act_foot_origin_rot, prev_ref_foot_origin_rot, target_foot_origin_rot, ref_foot_origin_rot;
  std::vector <hrp::Vector3> target_ee_p, rel_ee_pos, act_ee_p, projected_normal, act_force, ref_force, ref_moment;
  std::vector <hrp::Matrix33> target_ee_R, rel_ee_rot, act_ee_R, rel_ee_rot_for_ik;
  std::vector<std::string> rel_ee_name;
  rats::coordinates target_foot_midcoords;
  hrp::Vector3 ref_zmp, ref_cog, ref_cp, ref_cogvel, rel_ref_cp, prev_ref_cog, prev_ref_zmp;
  hrp::Vector3 act_zmp, act_cog, act_cogvel, act_cp, rel_act_zmp, rel_act_cp, prev_act_cog, act_base_rpy, current_base_rpy, current_base_pos, sbp_cog_offset, cp_offset, diff_cp, act_cmp;
  hrp::Vector3 foot_origin_offset[2];
  std::vector<double> prev_act_force_z;
  double zmp_origin_off, transition_smooth_gain, d_pos_z_root, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2], root_rot_compensation_limit[2];
  boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_cogvel_filter;
  OpenHRP::AutoBalancerService::STAlgorithm st_algorithm;
  SimpleZMPDistributor* szd;
  std::vector<std::vector<Eigen::Vector2d> > support_polygon_vetices, margined_support_polygon_vetices;
  // TPCC
  double k_tpcc_p[2], k_tpcc_x[2], d_rpy[2], k_brot_p[2], k_brot_tc[2];
  // RUN ST
  TwoDofController m_tau_x[2], m_tau_y[2], m_f_z;
  hrp::Vector3 pdr;
  double m_torque_k[2], m_torque_d[2]; // 3D-LIP parameters (0: x, 1: y)
  double pangx_ref, pangy_ref, pangx, pangy;
  double k_run_b[2], d_run_b[2];
  double rdx, rdy, rx, ry;
  // EEFM ST
  double eefm_k1[2], eefm_k2[2], eefm_k3[2], eefm_zmp_delay_time_const[2], eefm_body_attitude_control_gain[2], eefm_body_attitude_control_time_const[2];
  double eefm_pos_time_const_swing, eefm_pos_transition_time, eefm_pos_margin_time, eefm_gravitational_acceleration;
  std::vector<double> eefm_swing_damping_force_thre, eefm_swing_damping_moment_thre;
  hrp::Vector3 new_refzmp, after_walking_refzmp, rel_cog, ref_zmp_aux, diff_foot_origin_ext_moment;
  hrp::Vector3 pos_ctrl;
  hrp::Vector3 ref_total_force, ref_total_moment;
  // Total foot moment around the foot origin coords (relative to foot origin coords)
  hrp::Vector3 ref_total_foot_origin_moment, act_total_foot_origin_moment;
  hrp::Vector3 eefm_swing_pos_damping_gain, eefm_swing_rot_damping_gain;
  double swing2landing_transition_time, landing_phase_time, landing2support_transition_time, support_phase_min_time, support2swing_transition_time;
  double total_mass, transition_time, cop_check_margin, contact_decision_threshold;
  std::vector<double> cp_check_margin, tilt_margin;
  OpenHRP::AutoBalancerService::EmergencyCheckMode emergency_check_mode;
  hrp::dvector qRef, qCurrent, qRefSeq, controlSwingSupportTime, copInfo;
  hrp::Matrix33 baseRot;
  hrp::Vector3 zmpRef, basePos, rpy, baseRpy;
  std::vector<hrp::dvector>  wrenches, ref_wrenches;
  std::vector<hrp::Vector3>  limbCOPOffset;
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  bool use_act_states;
  std::vector<double> diff_q;
  interpolator *transition_interpolator;
  size_t falling_direction;
  std::map<std::string, interpolator*> swing_modification_interpolator;
  std::vector<bool> is_foot_touch;
  std::vector<hrp::Vector3> touchdown_d_pos, touchdown_d_rpy;
  bool use_force_sensor;
  // joint servo control
  OpenHRP::RobotHardwareService::JointControlMode joint_control_mode;
  RTC::CorbaConsumer<OpenHRP::RobotHardwareService> m_robotHardwareService0;
  bool is_reset_torque, is_after_walking;
  interpolator *after_walking_interpolator;
  bool use_footguided_stabilizer;
  double footguided_balance_time_const;
  size_t jump_time_count;
  double jump_initial_velocity;

  Stabilizer(hrp::BodyPtr& _robot, const std::string& _print_str, const double& _dt)
    : m_robot(_robot), print_str(_print_str), dt(_dt),
      control_mode(MODE_IDLE),
      st_algorithm(OpenHRP::AutoBalancerService::TPCC),
      emergency_check_mode(OpenHRP::AutoBalancerService::NO_CHECK),
      szd(NULL), joint_control_mode(OpenHRP::RobotHardwareService::POSITION),
      m_debugLevel(0)
  {
  };
  ~Stabilizer() {};

  void initStabilizer(const RTC::Properties& prop, const size_t& _num);
  void execStabilizer();
  void getCurrentParameters ();
  void getTargetParameters ();
  void getActualParameters ();
  void getActualParametersForST ();
  void waitSTTransition();
  void sync_2_st ();
  void sync_2_idle();
  void stopSTEmergency();
  inline int calcMaxTransitionCount ()
  {
      return (transition_time / dt);
  };
  void startStabilizer(void);
  void stopStabilizer(void);
  double calcDampingControl (const double tau_d, const double tau, const double prev_d,
                             const double DD, const double TT);
  hrp::Vector3 calcDampingControl (const hrp::Vector3& prev_d, const hrp::Vector3& TT);
  double calcDampingControl (const double prev_d, const double TT);
  hrp::Vector3 calcDampingControl (const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                   const hrp::Vector3& DD, const hrp::Vector3& TT);
  void calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p);
  void setSwingSupportJointServoGains();
  void calcExternalForce (const hrp::Vector3& cog, const hrp::Vector3& zmp, const hrp::Matrix33& rot);
  void calcTorque (const hrp::Matrix33& rot);
  void calcRUNST();
  void moveBasePosRotForBodyRPYControl ();
  double vlimit(double value, double llimit_value, double ulimit_value);
  hrp::Vector3 vlimit(const hrp::Vector3& value, double llimit_value, double ulimit_value);
  hrp::Vector3 vlimit(const hrp::Vector3& value, const hrp::Vector3& limit_value);
  hrp::Vector3 vlimit(const hrp::Vector3& value, const hrp::Vector3& llimit_value, const hrp::Vector3& ulimit_value);
  void calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot);
  bool calcZMP(hrp::Vector3& ret_zmp, const double zmp_z);
  void calcStateForEmergencySignal();
  void calcSwingSupportLimbGain();
  void calcTPCC();
  void calcEEForceMomentControl();
  void calcSwingEEModification ();
  void limbStretchAvoidanceControl (const std::vector<hrp::Vector3>& ee_p, const std::vector<hrp::Matrix33>& ee_R);
  void setBoolSequenceParam (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name);
  void setBoolSequenceParamWithCheckContact (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name);
  std::string getStabilizerAlgorithmString (OpenHRP::AutoBalancerService::STAlgorithm _st_algorithm);
  inline bool isContact (const size_t idx) // 0 = right, 1 = left
  {
    return (prev_act_force_z[idx] > 25.0);
  };
  void calcDiffFootOriginExtMoment ();
};

#endif // STABILIZER_COMPONENT_H
