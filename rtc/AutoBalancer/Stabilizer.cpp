// -*- C++ -*-
/*!
 * @file  Stabilizer.cpp
 * @brief stabilizer filter
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "Stabilizer.h"
#include "hrpsys/util/VectorConvert.h"
#include <math.h>
#include <boost/lambda/lambda.hpp>

typedef coil::Guard<coil::Mutex> Guard;

#ifndef deg2rad
#define deg2rad(x) ((x) * M_PI / 180.0)
#endif
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
#define DEBUGP2 (loop%10==0)

void Stabilizer::initStabilizer(const RTC::Properties& prop, const size_t& num)
{
  std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
  readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(print_str));
  if (interlocking_joints.size() > 0) {
    for (size_t i = 0; i < jpe_v.size(); i++) {
      std::cerr << "[" << print_str << "] Interlocking Joints for [" << stikp[i].ee_name << "]" << std::endl;
      jpe_v[i]->setInterlockingJointPairIndices(interlocking_joints, std::string(print_str));
    }
  }

  // parameters for TPCC
  act_zmp = hrp::Vector3::Zero();
  for (int i = 0; i < 2; i++) {
    k_tpcc_p[i] = 0.2;
    k_tpcc_x[i] = 4.0;
    k_brot_p[i] = 0.1;
    k_brot_tc[i] = 1.5;
  }
  // parameters for EEFM
  double k_ratio = 0.9;
  for (int i = 0; i < 2; i++) {
    eefm_k1[i] = -1.41429*k_ratio;
    eefm_k2[i] = -0.404082*k_ratio;
    eefm_k3[i] = -0.18*k_ratio;
    eefm_body_attitude_control_gain[i] = 0.5;
    eefm_body_attitude_control_time_const[i] = 1e5;
  }
  for (size_t i = 0; i < stikp.size(); i++) {
    STIKParam& ikp = stikp[i];
    hrp::Link* root = m_robot->link(ikp.target_name);
    ikp.eefm_rot_damping_gain = hrp::Vector3(20*5, 20*5, 1e5);
    ikp.eefm_rot_time_const = hrp::Vector3(1.5, 1.5, 1.5);
    ikp.eefm_rot_compensation_limit = deg2rad(10.0);
    ikp.eefm_swing_rot_spring_gain = hrp::Vector3(0.0, 0.0, 0.0);
    ikp.eefm_swing_rot_time_const = hrp::Vector3(1.5, 1.5, 1.5);
    ikp.eefm_pos_damping_gain = hrp::Vector3(3500*10, 3500*10, 3500);
    ikp.eefm_pos_time_const_support = hrp::Vector3(1.5, 1.5, 1.5);
    ikp.eefm_pos_compensation_limit = 0.025;
    ikp.eefm_swing_pos_spring_gain = hrp::Vector3(0.0, 0.0, 0.0);
    ikp.eefm_swing_pos_time_const = hrp::Vector3(1.5, 1.5, 1.5);
    ikp.eefm_ee_moment_limit = hrp::Vector3(1e4, 1e4, 1e4); // Default limit [Nm] is too large. Same as no limit.
    ikp.touchoff_remain_time = 0.0;
    if (ikp.ee_name.find("leg") == std::string::npos) { // Arm default
      ikp.eefm_ee_forcemoment_distribution_weight = Eigen::Matrix<double, 6,1>::Zero();
    } else { // Leg default
      for (size_t j = 0; j < 3; j++) {
        ikp.eefm_ee_forcemoment_distribution_weight[j] = 1; // Force
        ikp.eefm_ee_forcemoment_distribution_weight[j+3] = 1e-2; // Moment
      }
    }
    ikp.max_limb_length = 0.0;
    while (!root->isRoot()) {
      ikp.max_limb_length += root->b.norm();
      ikp.parent_name = root->name;
      root = root->parent;
    }
    ikp.limb_length_margin = 0.13;
    ikp.support_time = 0.0;
  }
  eefm_swing_rot_damping_gain = hrp::Vector3(20*5, 20*5, 1e5);
  eefm_swing_pos_damping_gain = hrp::Vector3(33600, 33600, 7000);
  eefm_pos_time_const_swing = 0.08;
  eefm_pos_transition_time = 0.01;
  eefm_pos_margin_time = 0.02;
  eefm_zmp_delay_time_const[0] = eefm_zmp_delay_time_const[1] = 0.055;
  //eefm_leg_inside_margin = 0.065; // [m]
  //eefm_leg_front_margin = 0.05;
  //eefm_leg_rear_margin = 0.05;
  //fm_wrench_alpha_blending = 1.0; // fz_alpha
  eefm_gravitational_acceleration = 9.80665; // [m/s^2]
  cop_check_margin = 20.0*1e-3; // [m]
  cp_check_margin.resize(4, 30*1e-3); // [m]
  cp_offset = hrp::Vector3(0.0, 0.0, 0.0); // [m]
  tilt_margin.resize(2, 30 * M_PI / 180); // [rad]
  contact_decision_threshold = 50; // [N]
  eefm_use_force_difference_control = true;
  eefm_use_swing_damping = false;
  eefm_swing_damping_force_thre.resize(3, 300);
  eefm_swing_damping_moment_thre.resize(3, 15);
  initial_cp_too_large_error = true;
  is_walking = false;
  is_single_walking = false;
  is_estop_while_walking = false;
  sbp_cog_offset = hrp::Vector3(0.0, 0.0, 0.0);
  use_limb_stretch_avoidance = false;
  use_zmp_truncation = false;
  limb_stretch_avoidance_time_const = 1.5;
  limb_stretch_avoidance_vlimit[0] = -100 * 1e-3 * dt; // lower limit
  limb_stretch_avoidance_vlimit[1] = 50 * 1e-3 * dt; // upper limit
  root_rot_compensation_limit[0] = root_rot_compensation_limit[1] = deg2rad(90.0);
  detection_count_to_air = static_cast<int>(0.0 / dt);
  transition_interpolator = new interpolator(1, dt, interpolator::HOFFARBIB, 1);
  transition_interpolator->setName(std::string(print_str)+" transition_interpolator");
  is_foot_touch.resize(stikp.size(), true);
  touchdown_d_pos.resize(stikp.size(), hrp::Vector3::Zero());
  touchdown_d_rpy.resize(stikp.size(), hrp::Vector3::Zero());
  prev_ref_zmp = hrp::Vector3::Zero();
  prev_ref_cog = hrp::Vector3::Zero();
  act_cogvel = hrp::Vector3::Zero();
  prev_act_foot_origin_rot = hrp::Matrix33::Identity();
  prev_act_foot_origin_pos = hrp::Vector3::Zero();
  swing2landing_transition_time = 0.05;
  landing_phase_time = 0.1;
  landing2support_transition_time = 0.5;
  support_phase_min_time = 0.1;
  support2swing_transition_time = 0.05;
  use_force_sensor = true;
  is_reset_torque = false;
  is_after_walking = false;
  after_walking_interpolator = new interpolator(1, dt, interpolator::HOFFARBIB, 1);
  after_walking_interpolator->setName(std::string(print_str)+" after_walking_interpolator");
  use_footguided_stabilizer = true;
  footguided_balance_time_const = 0.4; // [s]

  // parameters for RUNST
  double ke = 0, tc = 0;
  for (int i = 0; i < 2; i++) {
    m_tau_x[i].setup(ke, tc, dt);
    m_tau_x[i].setErrorPrefix(std::string(print_str));
    m_tau_y[i].setup(ke, tc, dt);
    m_tau_y[i].setErrorPrefix(std::string(print_str));
    m_f_z.setup(ke, tc, dt);
    m_f_z.setErrorPrefix(std::string(print_str));
  }
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  pdr = hrp::Vector3::Zero();

  // Check is legged robot or not
  is_legged_robot = false;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
    hrp::Sensor* sen= m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
    if ( sen != NULL ) is_legged_robot = true;
  }
  is_emergency_motion = is_emergency_step = is_emergency = false;
  reset_emergency_flag = false;

  // m_tau.data.length(m_robot->numJoints());
  transition_joint_q.resize(m_robot->numJoints());
  qorg.resize(m_robot->numJoints());
  qrefv.resize(m_robot->numJoints());
  transition_count = 0;
  loop = 0;
  m_is_falling_counter = 0;
  is_air_counter = 0;
  total_mass = m_robot->totalMass();
  ref_zmp_aux = hrp::Vector3::Zero();

  qRef.resize(m_robot->numJoints());
  qCurrent.resize(m_robot->numJoints());
  diff_q.resize(m_robot->numJoints());
  qRefSeq.resize(m_robot->numJoints());
  controlSwingSupportTime.resize(num, 1.0);
  copInfo.resize(num*3, 1.0); // nx, ny, fz for each end-effectors
  // m_actContactStates.data.length(m_contactStates.data.length());
  for (size_t i = 0; i < num; i++) {
    target_ee_p.push_back(hrp::Vector3::Zero());
    target_ee_R.push_back(hrp::Matrix33::Identity());
    act_ee_p.push_back(hrp::Vector3::Zero());
    act_ee_R.push_back(hrp::Matrix33::Identity());
    projected_normal.push_back(hrp::Vector3::Zero());
    act_force.push_back(hrp::Vector3::Zero());
    ref_force.push_back(hrp::Vector3::Zero());
    ref_moment.push_back(hrp::Vector3::Zero());
    prev_act_force_z.push_back(0.0);
    ref_contact_states.push_back(true);
    prev_ref_contact_states.push_back(true);
    // m_actContactStates.data[i] = false;
    act_contact_states.push_back(false);
    prev_act_contact_states.push_back(false);
    toeheel_ratio.push_back(1.0);
  }
  transition_time = 2.0;
  foot_origin_offset[0] = hrp::Vector3::Zero();
  foot_origin_offset[1] = hrp::Vector3::Zero();

  act_cogvel_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, dt, hrp::Vector3::Zero())); // [Hz]

  // // for debug output
  // m_originRefZmp.data.x = m_originRefZmp.data.y = m_originRefZmp.data.z = 0.0;
  // m_originRefCog.data.x = m_originRefCog.data.y = m_originRefCog.data.z = 0.0;
  // m_originRefCogVel.data.x = m_originRefCogVel.data.y = m_originRefCogVel.data.z = 0.0;
  // m_originNewZmp.data.x = m_originNewZmp.data.y = m_originNewZmp.data.z = 0.0;
  // m_originActZmp.data.x = m_originActZmp.data.y = m_originActZmp.data.z = 0.0;
  // m_originActCog.data.x = m_originActCog.data.y = m_originActCog.data.z = 0.0;
  // m_originActCogVel.data.x = m_originActCogVel.data.y = m_originActCogVel.data.z = 0.0;
  // m_allRefWrench.data.length(stikp.size() * 6); // 6 is wrench dim
  // m_allEEComp.data.length(stikp.size() * 6); // 6 is pos+rot dim
  // m_debugData.data.length(1); m_debugData.data[0] = 0.0;

  szd = new SimpleZMPDistributor(dt);
  std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
  for (size_t i = 0; i < stikp.size(); i++) {
    support_polygon_vec.push_back(std::vector<Eigen::Vector2d>(1,Eigen::Vector2d::Zero()));
  }
  szd->set_vertices(support_polygon_vec);

  rel_ee_pos.reserve(stikp.size());
  rel_ee_rot.reserve(stikp.size());
  rel_ee_name.reserve(stikp.size());
  rel_ee_rot_for_ik.reserve(stikp.size());

  hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
  if (sen == NULL) {
    std::cerr << "[" << print_str << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
  }
}

void Stabilizer::execStabilizer()
{
  loop++;

  if (is_legged_robot) {
    getTargetParameters();
    getActualParametersForST();
    calcStateForEmergencySignal();
    switch (control_mode) {
    case MODE_IDLE:
      if (!is_reset_torque) {
        if ( joint_control_mode == OpenHRP::RobotHardwareService::TORQUE ) {
          double tmp_time = 3.0;
          m_robotHardwareService0->setServoPGainPercentageWithTime("all",100,tmp_time);
          m_robotHardwareService0->setServoDGainPercentageWithTime("all",100,tmp_time);
          usleep(tmp_time * 1e6);
          m_robotHardwareService0->setServoTorqueGainPercentage("all",0);
          is_reset_torque = true;
        }
      }
      break;
    case MODE_AIR:
      if ( transition_count == 0 && on_ground ) sync_2_st();
      break;
    case MODE_ST:
      if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
        calcEEForceMomentControl();
      } else {
        calcTPCC();
      }
      if ( transition_count == 0 && !on_ground ) {
        if (is_air_counter < detection_count_to_air) ++is_air_counter;
        else control_mode = MODE_SYNC_TO_AIR;
      } else is_air_counter = 0;
      break;
    case MODE_SYNC_TO_IDLE:
      sync_2_idle();
      control_mode = MODE_IDLE;
      break;
    case MODE_SYNC_TO_AIR:
      sync_2_idle();
      control_mode = MODE_AIR;
      break;
    }
    copy (ref_contact_states.begin(), ref_contact_states.end(), prev_ref_contact_states.begin());
    copy (act_contact_states.begin(), act_contact_states.end(), prev_act_contact_states.begin());
    if (!transition_interpolator->isEmpty()) {
        for (int i = 0; i < m_robot->numJoints(); i++ ) {
            diff_q[i] = m_robot->joint(i)->q - qRef[i];
        }
    }
    getCurrentParameters();
  }
}

void Stabilizer::getCurrentParameters ()
{
  current_root_p = m_robot->rootLink()->p;
  current_root_R = m_robot->rootLink()->R;
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qorg[i] = m_robot->joint(i)->q;
  }
}

void Stabilizer::getTargetParameters ()
{
  // Reference world frame =>
  // update internal robot model
  if ( transition_count == 0 ) {
    transition_smooth_gain = 1.0;
  } else {
    double max_transition_count = calcMaxTransitionCount();
    transition_smooth_gain = 1/(1+exp(-9.19*(((max_transition_count - std::fabs(transition_count)) / max_transition_count) - 0.5)));
  }
  if (transition_count > 0) {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = ( qRef[i] - transition_joint_q[i] ) * transition_smooth_gain + transition_joint_q[i];
    }
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = qRef[i];
    }
  }
  if (!transition_interpolator->isEmpty()) {
    double tmp_ratio;
    transition_interpolator->get(&tmp_ratio, true);
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = qRef[i] + tmp_ratio * diff_q[i];
    }
  }
  if ( transition_count < 0 ) {
    transition_count++;
  } else if ( transition_count > 0 ) {
    if ( transition_count == 1 ) {
      std::cerr << "[" << print_str << "] [" << "] Move to MODE_IDLE" << std::endl;
      reset_emergency_flag = true;
    }
    transition_count--;
  }
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qrefv[i] = m_robot->joint(i)->q;
  }
  m_robot->rootLink()->p = basePos;
  target_root_p = m_robot->rootLink()->p;
  target_root_R = hrp::rotFromRpy(baseRpy);
  m_robot->rootLink()->R = target_root_R;
  m_robot->calcForwardKinematics();
  ref_zmp = m_robot->rootLink()->R * zmpRef + m_robot->rootLink()->p; // base frame -> world frame
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
  if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
    // apply inverse system
    hrp::Vector3 tmp_ref_zmp = ref_zmp + eefm_zmp_delay_time_const[0] * (ref_zmp - prev_ref_zmp) / dt;
    prev_ref_zmp = ref_zmp;
    ref_zmp = tmp_ref_zmp;
  }
  ref_cog = m_robot->calcCM();
  ref_total_force = hrp::Vector3::Zero();
  ref_total_moment = hrp::Vector3::Zero(); // Total moment around reference ZMP tmp
  ref_total_foot_origin_moment = hrp::Vector3::Zero();
  for (size_t i = 0; i < stikp.size(); i++) {
    hrp::Link* target = m_robot->link(stikp[i].target_name);
    //target_ee_p[i] = target->p + target->R * stikp[i].localCOPPos;
    target_ee_p[i] = target->p + target->R * stikp[i].localp;
    target_ee_R[i] = target->R * stikp[i].localR;
    ref_force[i] = hrp::Vector3(ref_wrenches[i][0], ref_wrenches[i][1], ref_wrenches[i][2]);
    ref_moment[i] = hrp::Vector3(ref_wrenches[i][3], ref_wrenches[i][4], ref_wrenches[i][5]);
    stikp[i].ref_force = ref_force[i];
    stikp[i].ref_moment = ref_moment[i];
    ref_total_force += ref_force[i];
    // Force/moment diff control
    ref_total_moment += (target_ee_p[i]-ref_zmp).cross(ref_force[i]);
    // Force/moment control
    // ref_total_moment += (target_ee_p[i]-ref_zmp).cross(hrp::Vector3(m_ref_wrenches[i].data[0], m_ref_wrenches[i].data[1], m_ref_wrenches[i].data[2]))
    //     + hrp::Vector3(m_ref_wrenches[i].data[3], m_ref_wrenches[i].data[4], m_ref_wrenches[i].data[5]);
    if (is_feedback_control_enable[i]) {
      ref_total_foot_origin_moment += (target_ee_p[i]-foot_origin_pos).cross(ref_force[i]) + ref_moment[i];
    }
  }
  // <= Reference world frame

  // Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST) because the coordinates for ref_cog differs among st algorithms.
  if (transition_count == (-1 * calcMaxTransitionCount() + 1)) { // max transition count. In MODE_IDLE => MODE_ST, transition_count is < 0 and upcounter. "+ 1" is upcount at the beginning of this function.
    prev_ref_cog = ref_cog;
    std::cerr << "[" << print_str << "]   Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST)." << std::endl;
  }

  if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
    // Reference foot_origin frame =>
    // initialize for new_refzmp
    new_refzmp = ref_zmp;
    rel_cog = m_robot->rootLink()->R.transpose() * (ref_cog-m_robot->rootLink()->p);
    ref_cogvel = (ref_cog - prev_ref_cog)/dt;
    prev_ref_cog = ref_cog;
    // convert world (current-tmp) => local (foot_origin)
    zmp_origin_off = ref_zmp(2) - foot_origin_pos(2);
    ref_zmp = foot_origin_rot.transpose() * (ref_zmp - foot_origin_pos);
    ref_cog = foot_origin_rot.transpose() * (ref_cog - foot_origin_pos);
    new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
    ref_cogvel = foot_origin_rot.transpose() * ref_cogvel;
    ref_foot_origin_rot = foot_origin_rot;
    ref_foot_origin_pos = foot_origin_pos;
    for (size_t i = 0; i < stikp.size(); i++) {
      stikp[i].target_ee_diff_p = foot_origin_rot.transpose() * (target_ee_p[i] - foot_origin_pos);
      stikp[i].ref_theta = Eigen::AngleAxisd(foot_origin_rot.transpose() * target_ee_R[i]);
      ref_force[i] = foot_origin_rot.transpose() * ref_force[i];
      ref_moment[i] = foot_origin_rot.transpose() * ref_moment[i];
    }
    ref_total_foot_origin_moment = foot_origin_rot.transpose() * ref_total_foot_origin_moment;
    ref_total_force = foot_origin_rot.transpose() * ref_total_force;
    ref_total_moment = foot_origin_rot.transpose() * ref_total_moment;
    target_foot_origin_rot = foot_origin_rot;
    // capture point
    ref_cp = ref_cog + ref_cogvel / std::sqrt(eefm_gravitational_acceleration / (ref_cog - ref_zmp)(2));
    rel_ref_cp = hrp::Vector3(ref_cp(0), ref_cp(1), ref_zmp(2));
    rel_ref_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_ref_cp) - m_robot->rootLink()->p);
    sbp_cog_offset = foot_origin_rot.transpose() * sbp_cog_offset;
    // <= Reference foot_origin frame
  } else {
    ref_cogvel = (ref_cog - prev_ref_cog)/dt;
  } // st_algorithm == OpenHRP::AutoBalancerService::EEFM
  // Calc swing support limb gain param
  calcSwingSupportLimbGain();
}

void Stabilizer::getActualParameters ()
{
  // Actual world frame =>
  if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
    // update by current joint angles
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = qCurrent[i];
    }
    // tempolary
    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot->calcForwardKinematics();
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(rpy));
    //hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r*0.5, m_rpy.data.p*0.5, m_rpy.data.y*0.5));
    act_root_R = m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics();
    act_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ) {
      m_robot->joint(i)->q = qorg[i];
    }
    m_robot->rootLink()->p = current_root_p;
    m_robot->rootLink()->R = current_root_R;
    m_robot->calcForwardKinematics();
  }
  // cog
  act_cog = m_robot->calcCM();
  // zmp
  on_ground = false;
  if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
    on_ground = calcZMP(act_zmp, zmp_origin_off+foot_origin_pos(2));
  } else {
    on_ground = calcZMP(act_zmp, ref_zmp(2));
  }
  // set actual contact states
  for (size_t i = 0; i < stikp.size(); i++) {
    std::string limb_name = stikp[i].ee_name;
    size_t idx = contact_states_index_map[limb_name];
    act_contact_states[idx] = (use_force_sensor || limb_name.find("leg") == std::string::npos) ? isContact(idx) : ref_contact_states[idx];
    // m_actContactStates.data[idx] = act_contact_states[idx];
  }
  // <= Actual world frame

  // convert absolute (in st) -> root-link relative
  rel_act_zmp = m_robot->rootLink()->R.transpose() * (act_zmp - m_robot->rootLink()->p);
  if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
    // Actual foot_origin frame =>
    act_zmp = foot_origin_rot.transpose() * (act_zmp - foot_origin_pos);
    act_cog = foot_origin_rot.transpose() * (act_cog - foot_origin_pos);
    if (!use_force_sensor) {
      on_ground = true;
      act_zmp = ref_zmp;
    }
    //act_cogvel = foot_origin_rot.transpose() * act_cogvel;
    if ((foot_origin_pos - prev_act_foot_origin_pos).norm() > 1e-2) { // assume that origin_pos changes more than 1cm when contact states change
      act_cogvel = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * act_cogvel;
    } else {
      if (act_contact_states[contact_states_index_map["rleg"]] || act_contact_states[contact_states_index_map["lleg"]]) { // on ground
          act_cogvel = (act_cog - prev_act_cog)/dt;
      } else if (prev_act_contact_states[contact_states_index_map["rleg"]] || prev_act_contact_states[contact_states_index_map["lleg"]]) { // take off
        jump_time_count = 1;
        jump_initial_velocity = act_cogvel(2);
        act_cogvel(2) = jump_initial_velocity - eefm_gravitational_acceleration * jump_time_count * dt;
      } else { // jumping
        jump_time_count++;
        act_cogvel(2) = jump_initial_velocity - eefm_gravitational_acceleration * jump_time_count * dt;
      }
    }
    act_cogvel = act_cogvel_filter->passFilter(act_cogvel);
    prev_act_cog = act_cog;
    //act_root_rot = m_robot->rootLink()->R;
    for (size_t i = 0; i < stikp.size(); i++) {
      hrp::Link* target = m_robot->link(stikp[i].target_name);
      //hrp::Vector3 act_ee_p = target->p + target->R * stikp[i].localCOPPos;
      hrp::Vector3 _act_ee_p = target->p + target->R * stikp[i].localp;
      act_ee_p[i] = foot_origin_rot.transpose() * (_act_ee_p - foot_origin_pos);
      act_ee_R[i] = foot_origin_rot.transpose() * (target->R * stikp[i].localR);
    }
    // capture point
    act_cp = act_cog + act_cogvel / std::sqrt(eefm_gravitational_acceleration / (act_cog - act_zmp)(2));
    rel_act_cp = hrp::Vector3(act_cp(0), act_cp(1), act_zmp(2));
    rel_act_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_act_cp) - m_robot->rootLink()->p);
    // <= Actual foot_origin frame
  }
  // calc cmp
  hrp::Vector3 total_force = hrp::Vector3::Zero();
  for (size_t i = 0; i < stikp.size(); i++) { // TODO remove duplicatation in getActualParametersForST
    STIKParam& ikp = stikp[i];
    if (!is_feedback_control_enable[i]) continue;
    hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(ikp.sensor_name);
    total_force += foot_origin_rot.transpose() * ((sensor->link->R * sensor->localR) * hrp::Vector3(wrenches[i][0], wrenches[i][1], wrenches[i][2]));
  }
  double tmp_k = (act_cog - act_zmp)(2) / total_force(2);
  act_cmp.head(2) = (act_cog - tmp_k * total_force).head(2);
  act_cmp(2) = act_zmp(2);
}

void Stabilizer::getActualParametersForST ()
{
  // Actual world frame =>
  if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
    // update by current joint angles
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = qCurrent[i];
    }
    // tempolary
    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot->rootLink()->R = act_root_R;
    m_robot->calcForwardKinematics();
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ) {
      m_robot->joint(i)->q = qorg[i];
    }
    m_robot->rootLink()->p = current_root_p;
    m_robot->rootLink()->R = current_root_R;
    m_robot->calcForwardKinematics();
  }

  if (st_algorithm != OpenHRP::AutoBalancerService::TPCC) {
    // Actual world frame =>
    // new ZMP calculation
    // Kajita's feedback law
    //   Basically Equation (26) in the paper [1].
    new_refzmp = foot_origin_rot * new_refzmp + foot_origin_pos; // = abs_ref_zmp
    if (!is_walking || !use_act_states) {
      if (use_footguided_stabilizer && use_act_states) { // guided_stabilzier is not valid while walking
        double omega = std::sqrt(eefm_gravitational_acceleration / (act_cog - act_zmp)(2));
        hrp::Vector3 act_xcp = (foot_origin_pos + foot_origin_rot * (act_cp + sbp_cog_offset)) - new_refzmp;
        hrp::Vector3 dxsp = hrp::Vector3::Zero(); // stay still
        new_refzmp += transition_smooth_gain * 2 * (act_xcp - std::exp(- omega * footguided_balance_time_const) * dxsp) / (1 - std::exp(-2 * omega * footguided_balance_time_const));
      } else { // kajita st
        hrp::Vector3 dcog=foot_origin_rot * (ref_cog - act_cog);
        hrp::Vector3 dcogvel=foot_origin_rot * (ref_cogvel - act_cogvel);
        hrp::Vector3 dzmp=foot_origin_rot * (ref_zmp - act_zmp);
        for (size_t i = 0; i < 2; i++) {
          new_refzmp(i) += eefm_k1[i] * transition_smooth_gain * dcog(i) + eefm_k2[i] * transition_smooth_gain * dcogvel(i) + eefm_k3[i] * transition_smooth_gain * dzmp(i) + ref_zmp_aux(i);
        }
      }
    }
    if (use_act_states && !use_footguided_stabilizer) {
      if (is_walking) {
        is_after_walking = true;
        if (!after_walking_interpolator->isEmpty()) after_walking_interpolator->clear();
        after_walking_refzmp = new_refzmp;
      } else {
        if (is_after_walking) {
          double tmp_ratio = 1.0;
          after_walking_interpolator->set(&tmp_ratio);
          tmp_ratio = 0.0;
          after_walking_interpolator->setGoal(&tmp_ratio, 0.01, true);
          is_after_walking = false;
        }
        if (!after_walking_interpolator->isEmpty()) {
          double tmp_ratio;
          after_walking_interpolator->get(&tmp_ratio, true);
          new_refzmp = tmp_ratio * after_walking_refzmp + (1.0 - tmp_ratio) * new_refzmp;
        }
      }
    }
    if (DEBUGP) {
      // All state variables are foot_origin coords relative
      std::cerr << "[" << print_str << "] state values" << std::endl;
      std::cerr << "[" << print_str << "]   "
                << "ref_cog    = " << hrp::Vector3(ref_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_cog    = " << hrp::Vector3(act_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
      std::cerr << "[" << print_str << "]   "
                << "ref_cogvel = " << hrp::Vector3(ref_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_cogvel = " << hrp::Vector3(act_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm/s]" << std::endl;
      std::cerr << "[" << print_str << "]   "
                << "ref_zmp    = " << hrp::Vector3(ref_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_zmp    = " << hrp::Vector3(act_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
      hrp::Vector3 tmpnew_refzmp;
      tmpnew_refzmp = foot_origin_rot.transpose()*(new_refzmp-foot_origin_pos); // Actual world -> foot origin relative
      std::cerr << "[" << print_str << "]   "
                << "new_zmp    = " << hrp::Vector3(tmpnew_refzmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", dif_zmp    = " << hrp::Vector3((tmpnew_refzmp-ref_zmp)*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
    }

    std::vector<std::string> ee_name;
    // distribute new ZMP into foot force & moment
    std::vector<hrp::Vector3> tmp_ref_force, tmp_ref_moment;
    std::vector<double> limb_gains;
    std::vector<hrp::dvector6> ee_forcemoment_distribution_weight;
    std::vector<double> tmp_toeheel_ratio;
    double tmp_total_fz = 0.0;
    if (control_mode == MODE_ST) {
      std::vector<hrp::Vector3> ee_pos, cop_pos;
      std::vector<hrp::Matrix33> ee_rot;
      std::vector<bool> is_contact_list;
      is_contact_list.reserve(stikp.size());
      for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        if (!is_ik_enable[i]) continue;
        hrp::Link* target = m_robot->link(ikp.target_name);
        rel_ee_rot_for_ik.push_back(foot_origin_rot.transpose() * (target->R * ikp.localR));
        ee_pos.push_back(target->p + target->R * ikp.localp);
        cop_pos.push_back(target->p + target->R * ikp.localCOPPos);
        ee_rot.push_back(target->R * ikp.localR);
        limb_gains.push_back(ikp.swing_support_gain);
        tmp_ref_force.push_back(hrp::Vector3(foot_origin_rot * ref_force[i]));
        tmp_ref_moment.push_back(hrp::Vector3(foot_origin_rot * ref_moment[i]));
        rel_ee_pos.push_back(foot_origin_rot.transpose() * (ee_pos.back() - foot_origin_pos));
        rel_ee_rot.push_back(foot_origin_rot.transpose() * ee_rot.back());
        if (!is_feedback_control_enable[i]) continue;
        ee_name.push_back(ikp.ee_name);
        rel_ee_name.push_back(ee_name.back());
        is_contact_list.push_back(act_contact_states[i]);
        // std::cerr << ee_forcemoment_distribution_weight[i] << std::endl;
        ee_forcemoment_distribution_weight.push_back(hrp::dvector6::Zero(6,1));
        for (size_t j = 0; j < 6; j++) {
          ee_forcemoment_distribution_weight[i][j] = ikp.eefm_ee_forcemoment_distribution_weight[j];
        }
        tmp_toeheel_ratio.push_back(toeheel_ratio[i]);
        tmp_total_fz += tmp_ref_force.back()(2);
      }

      // All state variables are foot_origin coords relative
      if (DEBUGP) {
        std::cerr << "[" << print_str << "] ee values" << std::endl;
        hrp::Vector3 tmpp;
        for (size_t i = 0; i < ee_name.size(); i++) {
          tmpp = foot_origin_rot.transpose()*(ee_pos[i]-foot_origin_pos);
          std::cerr << "[" << print_str << "]   "
                    << "ee_pos (" << ee_name[i] << ")    = " << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"));
          tmpp = foot_origin_rot.transpose()*(cop_pos[i]-foot_origin_pos);
          std::cerr << ", cop_pos (" << ee_name[i] << ")    = " << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
        }
      }

      // truncate ZMP
      if (use_zmp_truncation && !is_walking) {
        Eigen::Vector2d tmp_new_refzmp(new_refzmp.head(2));
        szd->get_vertices(support_polygon_vetices);
        szd->calc_convex_hull(support_polygon_vetices, ref_contact_states, ee_pos, ee_rot);
        if (!szd->is_inside_support_polygon(tmp_new_refzmp, hrp::Vector3::Zero(), true, std::string(print_str))) new_refzmp.head(2) = tmp_new_refzmp;
      }

      // Distribute ZMP into each EE force/moment at each COP
      if (on_ground) {
        if (st_algorithm == OpenHRP::AutoBalancerService::EEFM) {
        // Modified version of distribution in Equation (4)-(6) and (10)-(13) in the paper [1].
        szd->distributeZMPToForceMoments(tmp_ref_force, tmp_ref_moment,
                                         ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                         new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                         tmp_total_fz, dt,
                                         DEBUGP, std::string(print_str));
      } else if (st_algorithm == OpenHRP::AutoBalancerService::EEFMQP) {
        szd->distributeZMPToForceMomentsQP(tmp_ref_force, tmp_ref_moment,
                                           ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                           new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                           tmp_total_fz, dt,
                                           DEBUGP, std::string(print_str),
                                           (st_algorithm == OpenHRP::AutoBalancerService::EEFMQPCOP));
      } else if (st_algorithm == OpenHRP::AutoBalancerService::EEFMQPCOP) {
        szd->distributeZMPToForceMomentsPseudoInverse(tmp_ref_force, tmp_ref_moment,
                                                      ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                                      new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                      tmp_total_fz, dt,
                                                      DEBUGP, std::string(print_str),
                                                      (st_algorithm == OpenHRP::AutoBalancerService::EEFMQPCOP), is_contact_list);
      } else if (st_algorithm == OpenHRP::AutoBalancerService::EEFMQPCOP2) {
        szd->distributeZMPToForceMomentsPseudoInverse2(tmp_ref_force, tmp_ref_moment,
                                                       ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                                       new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                       foot_origin_rot * ref_total_force, foot_origin_rot * ref_total_moment,
                                                       ee_forcemoment_distribution_weight,
                                                       tmp_total_fz, dt,
                                                       DEBUGP, std::string(print_str));
      }
      }
      // for debug output
      new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
    }

    // foor modif
    if (control_mode == MODE_ST) {
      hrp::Vector3 f_diff(hrp::Vector3::Zero());
      std::vector<bool> large_swing_f_diff(3, false);
      // moment control
      act_total_foot_origin_moment = hrp::Vector3::Zero();
      for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        std::vector<bool> large_swing_m_diff(3, false);
        if (!is_feedback_control_enable[i]) continue;
        hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(ikp.sensor_name);
        hrp::Link* target = m_robot->link(ikp.target_name);
        // Convert moment at COP => moment at ee
        size_t idx = contact_states_index_map[ikp.ee_name];
        ikp.ref_moment = tmp_ref_moment[idx] + ((target->R * ikp.localCOPPos + target->p) - (target->R * ikp.localp + target->p)).cross(tmp_ref_force[idx]);
        ikp.ref_force = tmp_ref_force[idx];
        // Actual world frame =>
        hrp::Vector3 sensor_force = (sensor->link->R * sensor->localR) * hrp::Vector3(wrenches[i][0], wrenches[i][1], wrenches[i][2]);
        hrp::Vector3 sensor_moment = (sensor->link->R * sensor->localR) * hrp::Vector3(wrenches[i][3], wrenches[i][4], wrenches[i][5]);
        //hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localCOPPos + target->p)).cross(sensor_force) + sensor_moment;
        hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localp + target->p)).cross(sensor_force) + sensor_moment;
        // <= Actual world frame
        hrp::Matrix33 ee_R(target->R * ikp.localR);
        // Actual ee frame =>
        ikp.ref_moment = ee_R.transpose() * ikp.ref_moment;
        ikp.ref_force = ee_R.transpose() * ikp.ref_force;
        sensor_force = ee_R.transpose() * sensor_force;
        ee_moment = ee_R.transpose() * ee_moment;
        if ( i == 0 ) f_diff += -1*sensor_force;
        else f_diff += sensor_force;
        for (size_t j = 0; j < 3; ++j) {
          if ((!ref_contact_states[i] || !act_contact_states[i]) && fabs(ikp.ref_force(j) - sensor_force(j)) > eefm_swing_damping_force_thre[j]) large_swing_f_diff[j] = true;
          if ((!ref_contact_states[i] || !act_contact_states[i]) && (fabs(ikp.ref_moment(j) - ee_moment(j)) > eefm_swing_damping_moment_thre[j])) large_swing_m_diff[j] = true;
        }
        // Moment limitation
        ikp.ref_moment = ee_R * vlimit((ee_R.transpose() * ikp.ref_moment), ikp.eefm_ee_moment_limit);
        // calcDampingControl
        // ee_d_foot_rpy and ee_d_foot_pos is (actual) end effector coords relative value because these use end effector coords relative force & moment
        { // Rot
          //   Basically Equation (16) and (17) in the paper [1]
          hrp::Vector3 tmp_damping_gain;
          for (size_t j = 0; j < 3; ++j) {
            double tmp_damping = ikp.eefm_rot_damping_gain(j) * (is_single_walking ? 1.0 : 4.0);
            if (!eefm_use_swing_damping || !large_swing_m_diff[j]) tmp_damping_gain(j) = (1-transition_smooth_gain) * tmp_damping * 10 + transition_smooth_gain * tmp_damping;
            else tmp_damping_gain(j) = (1-transition_smooth_gain) * eefm_swing_rot_damping_gain(j) * 10 + transition_smooth_gain * eefm_swing_rot_damping_gain(j);
          }
          if (!ref_contact_states[i] && !act_contact_states[i])
            ikp.ee_d_foot_rpy = calcDampingControl(ikp.d_foot_rpy, ikp.eefm_rot_time_const);
          else
            ikp.ee_d_foot_rpy = calcDampingControl(ikp.ref_moment, ee_moment, ikp.ee_d_foot_rpy, tmp_damping_gain, ikp.eefm_rot_time_const);
          ikp.ee_d_foot_rpy = vlimit(ikp.ee_d_foot_rpy, -1 * ikp.eefm_rot_compensation_limit, ikp.eefm_rot_compensation_limit);
        }
        if (!eefm_use_force_difference_control) { // Pos
          hrp::Vector3 tmp_damping_gain = (1-transition_smooth_gain) * ikp.eefm_pos_damping_gain * 10 + transition_smooth_gain * ikp.eefm_pos_damping_gain;
          if (!ref_contact_states[i] && !act_contact_states[i])
            ikp.ee_d_foot_pos = calcDampingControl(ikp.d_foot_pos, ikp.eefm_pos_time_const_support);
          else
            ikp.ee_d_foot_pos = calcDampingControl(ikp.ref_force, sensor_force, ikp.ee_d_foot_pos, tmp_damping_gain, ikp.eefm_pos_time_const_support);
          ikp.ee_d_foot_pos = vlimit(ikp.ee_d_foot_pos, -1 * ikp.eefm_pos_compensation_limit, ikp.eefm_pos_compensation_limit);
        }
        // Convert force & moment as foot origin coords relative
        ikp.ref_moment = foot_origin_rot.transpose() * ee_R * ikp.ref_moment;
        ikp.ref_force = foot_origin_rot.transpose() * ee_R * ikp.ref_force;
        sensor_force = foot_origin_rot.transpose() * ee_R * sensor_force;
        ee_moment = foot_origin_rot.transpose() * ee_R * ee_moment;
        ikp.d_foot_rpy = foot_origin_rot.transpose() * ee_R * ikp.ee_d_foot_rpy;
        ikp.d_foot_pos = foot_origin_rot.transpose() * ee_R * ikp.ee_d_foot_pos;
        if ( joint_control_mode == OpenHRP::RobotHardwareService::TORQUE ) {
            ikp.d_foot_rpy = hrp::Vector3::Zero();
            ikp.d_foot_pos = hrp::Vector3::Zero();
        }
        // tilt Check : only flat plane is supported
        {
          hrp::Vector3 plane_x = target_ee_R[i].col(0);
          hrp::Vector3 plane_y = target_ee_R[i].col(1);
          hrp::Matrix33 act_ee_R_world = target->R * stikp[i].localR;
          hrp::Vector3 normal_vector = act_ee_R_world.col(2);
          /* projected_normal = c1 * plane_x + c2 * plane_y : c1 = plane_x.dot(normal_vector), c2 = plane_y.dot(normal_vector) because (normal-vector - projected_normal) is orthogonal to plane */
          projected_normal.at(i) = plane_x.dot(normal_vector) * plane_x + plane_y.dot(normal_vector) * plane_y;
          act_force.at(i) = sensor_force;
        }
        //act_total_foot_origin_moment += (target->R * ikp.localCOPPos + target->p).cross(sensor_force) + ee_moment;
        act_total_foot_origin_moment += (target->R * ikp.localp + target->p - foot_origin_pos).cross(sensor_force) + ee_moment;
      }
      act_total_foot_origin_moment = foot_origin_rot.transpose() * act_total_foot_origin_moment;

      if (eefm_use_force_difference_control) {
        // fxyz control
        // foot force difference control version
        //   Basically Equation (18) in the paper [1]
        hrp::Vector3 ref_f_diff = (stikp[1].ref_force-stikp[0].ref_force);
        if (ref_contact_states != prev_ref_contact_states) pos_ctrl = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * pos_ctrl;
        if (eefm_use_swing_damping) {
          hrp::Vector3 tmp_damping_gain;
          for (size_t i = 0; i < 3; ++i) {
            if (!large_swing_f_diff[i]) tmp_damping_gain(i) = (1-transition_smooth_gain) * stikp[0].eefm_pos_damping_gain(i) * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain(i);
            else tmp_damping_gain(i) = (1-transition_smooth_gain) * eefm_swing_pos_damping_gain(i) * 10 + transition_smooth_gain * eefm_swing_pos_damping_gain(i);
          }
          pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                         tmp_damping_gain, stikp[0].eefm_pos_time_const_support);
        } else {
          hrp::Vector3 tmp_damping = stikp[0].eefm_pos_damping_gain * (is_single_walking ? 1.0 : 4.0);
          if ( (ref_contact_states[contact_states_index_map["rleg"]] && ref_contact_states[contact_states_index_map["lleg"]]) // Reference : double support phase
               || (act_contact_states[0] && act_contact_states[1]) ) { // Actual : double support phase
            // Temporarily use first pos damping gain (stikp[0])
            hrp::Vector3 tmp_damping_gain = (1-transition_smooth_gain) * tmp_damping * 10 + transition_smooth_gain * tmp_damping;
            pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                           tmp_damping_gain, stikp[0].eefm_pos_time_const_support);
          } else {
            double remain_swing_time;
            if ( !ref_contact_states[contact_states_index_map["rleg"]] ) { // rleg swing
              remain_swing_time = controlSwingSupportTime[contact_states_index_map["rleg"]];
            } else { // lleg swing
              remain_swing_time = controlSwingSupportTime[contact_states_index_map["lleg"]];
            }
            // std::cerr << "st " << remain_swing_time << " rleg " << contact_states[contact_states_index_map["rleg"]] << " lleg " << contact_states[contact_states_index_map["lleg"]] << std::endl;
            double tmp_ratio = std::max(0.0, std::min(1.0, 1.0 - (remain_swing_time-eefm_pos_margin_time)/eefm_pos_transition_time)); // 0=>1
            // Temporarily use first pos damping gain (stikp[0])
            hrp::Vector3 tmp_damping_gain = (1-transition_smooth_gain) * tmp_damping * 10 + transition_smooth_gain * tmp_damping;
            hrp::Vector3 tmp_time_const = (1-tmp_ratio)*eefm_pos_time_const_swing*hrp::Vector3::Ones()+tmp_ratio*stikp[0].eefm_pos_time_const_support;
            pos_ctrl = calcDampingControl (tmp_ratio * ref_f_diff, tmp_ratio * f_diff, pos_ctrl, tmp_damping_gain, tmp_time_const);
          }
        }
        // zctrl = vlimit(zctrl, -0.02, 0.02);
        // Temporarily use first pos compensation limit (stikp[0])
        pos_ctrl = vlimit(pos_ctrl, -1 * stikp[0].eefm_pos_compensation_limit * 2, stikp[0].eefm_pos_compensation_limit * 2);
        // Divide pos_ctrl into rfoot and lfoot
        stikp[0].d_foot_pos = -0.5 * pos_ctrl;
        stikp[1].d_foot_pos = 0.5 * pos_ctrl;
      }
      if (DEBUGP) {
        std::cerr << "[" << print_str << "] Control values" << std::endl;
        if (eefm_use_force_difference_control) {
          std::cerr << "[" << print_str << "]   "
                    << "pos_ctrl    = [" << pos_ctrl(0)*1e3 << " " << pos_ctrl(1)*1e3 << " "<< pos_ctrl(2)*1e3 << "] [mm]" << std::endl;
        }
        for (size_t i = 0; i < ee_name.size(); i++) {
          std::cerr << "[" << print_str << "]   "
                    << "d_foot_pos (" << ee_name[i] << ")  = [" << stikp[i].d_foot_pos(0)*1e3 << " " << stikp[i].d_foot_pos(1)*1e3 << " " << stikp[i].d_foot_pos(2)*1e3 << "] [mm], "
                    << "d_foot_rpy (" << ee_name[i] << ")  = [" << stikp[i].d_foot_rpy(0)*180.0/M_PI << " " << stikp[i].d_foot_rpy(1)*180.0/M_PI << " " << stikp[i].d_foot_rpy(2)*180.0/M_PI << "] [deg]" << std::endl;
        }
      }
      // foot force independent damping control
      // for (size_t i = 0; i < 2; i++) {
      //   f_zctrl[i] = calcDampingControl (ref_force[i](2),
      //                                    fz[i], f_zctrl[i], eefm_pos_damping_gain, eefm_pos_time_const);
      //   f_zctrl[i] = vlimit(f_zctrl[i], -0.05, 0.05);
      // }
      calcDiffFootOriginExtMoment ();
    }
  } // st_algorithm == OpenHRP::AutoBalancerService::EEFM

  if ( joint_control_mode == OpenHRP::RobotHardwareService::TORQUE && control_mode == MODE_ST ) setSwingSupportJointServoGains();
  calcExternalForce(foot_origin_rot * act_cog + foot_origin_pos, foot_origin_rot * new_refzmp + foot_origin_pos, foot_origin_rot); // foot origin relative => Actual world frame
  calcTorque(foot_origin_rot);

  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = qrefv[i];
  }
  m_robot->rootLink()->p = target_root_p;
  m_robot->rootLink()->R = target_root_R;
  if ( !(control_mode == MODE_IDLE || control_mode == MODE_AIR) ) {
    for (size_t i = 0; i < jpe_v.size(); i++) {
      if (is_ik_enable[i]) {
        for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
          int idx = jpe_v[i]->joint(j)->jointId;
          m_robot->joint(idx)->q = qorg[idx];
        }
      }
    }
    m_robot->rootLink()->p(0) = current_root_p(0);
    m_robot->rootLink()->p(1) = current_root_p(1);
    m_robot->rootLink()->R = current_root_R;
    m_robot->calcForwardKinematics();
  }
  if (control_mode != MODE_ST) d_pos_z_root = 0.0;
  prev_act_foot_origin_rot = foot_origin_rot;
  prev_act_foot_origin_pos = foot_origin_pos;
}

void Stabilizer::waitSTTransition()
{
  // Wait condition
  //   1. Check transition_count : Wait until transition is finished
  //   2. Check control_mode : Once control_mode is SYNC mode, wait until control_mode moves to the next mode (MODE_AIR or MODE_IDLE)
  bool flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
  while (transition_count != 0 ||
         (flag ? !(control_mode == MODE_IDLE || control_mode == MODE_AIR) : false) ) {
    usleep(10);
    flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
  }
  usleep(10);
}

void Stabilizer::sync_2_st ()
{
  std::cerr << "[" << print_str << "] ["
            << "] Sync IDLE => ST"  << std::endl;
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  d_rpy[0] = d_rpy[1] = 0;
  pdr = hrp::Vector3::Zero();
  pos_ctrl = hrp::Vector3::Zero();
  prev_ref_foot_origin_rot = hrp::Matrix33::Identity();
  is_after_walking = false;
  if (!after_walking_interpolator->isEmpty()) after_walking_interpolator->clear();
  for (size_t i = 0; i < stikp.size(); i++) {
    STIKParam& ikp = stikp[i];
    ikp.target_ee_diff_p = hrp::Vector3::Zero();
    ikp.d_pos_swing = ikp.prev_d_pos_swing = hrp::Vector3::Zero();
    ikp.d_rpy_swing = ikp.prev_d_rpy_swing = hrp::Vector3::Zero();
    ikp.target_ee_diff_p_filter->reset(hrp::Vector3::Zero());
    ikp.d_foot_pos = ikp.ee_d_foot_pos = ikp.d_foot_rpy = ikp.ee_d_foot_rpy = hrp::Vector3::Zero();
    ikp.omega.angle() = 0.0;
    swing_modification_interpolator[ikp.ee_name]->clear();
    is_foot_touch[i] = true;
  }
  if (on_ground) {
    transition_count = -1 * calcMaxTransitionCount();
    control_mode = MODE_ST;
  } else {
    transition_count = 0;
    control_mode = MODE_AIR;
  }
}

void Stabilizer::sync_2_idle ()
{
  std::cerr << "[" << print_str << "] ["
            << "] Sync ST => IDLE"  << std::endl;
  transition_count = calcMaxTransitionCount();
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q;
  }
}

void Stabilizer::stopSTEmergency()
{
  std::cerr << "[" << print_str << "] stop stabilizer mode for emergency" << std::endl;
  for ( std::map<std::string, interpolator*>::iterator it = swing_modification_interpolator.begin(); it != swing_modification_interpolator.end(); it++ ) {
    it->second->clear();
  }
  double tmp_ratio = 1.0;
  transition_interpolator->clear();
  transition_interpolator->set(&tmp_ratio);
  tmp_ratio = 0.0;
  transition_interpolator->setGoal(&tmp_ratio, 0.8, true);
  // for (int i = 0; i < m_robot->numJoints(); i++ ) {
  //   diff_q[i] = qorg[i] - qRef[i];
  // }
  control_mode = MODE_IDLE;
}

void Stabilizer::startStabilizer(void)
{
  waitSTTransition(); // Wait until all transition has finished
  {
    Guard guard(m_mutex);
    if ( control_mode == MODE_IDLE ) {
      std::cerr << "[" << print_str << "] " << "Start ST"  << std::endl;
      sync_2_st();
    }
  }
  waitSTTransition();
  if ( joint_control_mode == OpenHRP::RobotHardwareService::TORQUE ) {
      std::cerr << "[" << print_str << "] " << "Moved to ST command pose and sync to TORQUE mode"  << std::endl;
      m_robotHardwareService0->setServoGainPercentage("all",100);//tmp
      m_robotHardwareService0->setServoTorqueGainPercentage("all",100);
      for(size_t i = 0; i < stikp.size(); i++) {
          STIKParam& ikp = stikp[i];
          hrp::JointPathExPtr jpe = jpe_v[i];
          for(size_t j = 0; j < ikp.support_pgain.size(); j++) {
              m_robotHardwareService0->setServoPGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.support_pgain(j),3);
              m_robotHardwareService0->setServoDGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.support_dgain(j),3);
          }
      }
      is_reset_torque = false;
  }
  std::cerr << "[" << print_str << "] " << "Start ST DONE"  << std::endl;
}

void Stabilizer::stopStabilizer(void)
{
  waitSTTransition(); // Wait until all transition has finished
  if ( joint_control_mode == OpenHRP::RobotHardwareService::TORQUE ) {
      double tmp_time = 3.0;
      m_robotHardwareService0->setServoPGainPercentageWithTime("all",100,tmp_time);
      m_robotHardwareService0->setServoDGainPercentageWithTime("all",100,tmp_time);
      usleep(tmp_time * 1e6);
      m_robotHardwareService0->setServoTorqueGainPercentage("all",0);
      is_reset_torque = true;
  }
  {
    Guard guard(m_mutex);
    if ( (control_mode == MODE_ST || control_mode == MODE_AIR) ) {
      std::cerr << "[" << print_str << "] " << "Stop ST"  << std::endl;
      control_mode = (control_mode == MODE_ST) ? MODE_SYNC_TO_IDLE : MODE_IDLE;
    }
  }
  waitSTTransition();
  std::cerr << "[" << print_str << "] " << "Stop ST DONE"  << std::endl;
}

// Damping control functions
//   Basically Equation (14) in the paper [1]
double Stabilizer::calcDampingControl (const double tau_d, const double tau, const double prev_d,
                                       const double DD, const double TT)
{
  return (1/DD * (tau_d - tau) - 1/TT * prev_d) * dt + prev_d;
};

// Retrieving only
hrp::Vector3 Stabilizer::calcDampingControl (const hrp::Vector3& prev_d, const hrp::Vector3& TT)
{
  return (- prev_d.cwiseQuotient(TT)) * dt + prev_d;
};

// Retrieving only
double Stabilizer::calcDampingControl (const double prev_d, const double TT)
{
  return - 1/TT * prev_d * dt + prev_d;
};

hrp::Vector3 Stabilizer::calcDampingControl (const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                             const hrp::Vector3& DD, const hrp::Vector3& TT)
{
  return ((tau_d - tau).cwiseQuotient(DD) - prev_d.cwiseQuotient(TT)) * dt + prev_d;
};

void Stabilizer::calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p)
{
  // tm.resize(6,6*contact_p.size());
  // tm.setZero();
  // for (size_t c = 0; c < contact_p.size(); c++) {
  //   for (size_t i = 0; i < 6; i++) tm(i,(c*6)+i) = 1.0;
  //   hrp::Matrix33 cm;
  //   rats::outer_product_matrix(cm, contact_p[c]);
  //   for (size_t i = 0; i < 3; i++)
  //     for (size_t j = 0; j < 3; j++) tm(i+3,(c*6)+j) = cm(i,j);
  // }
}

void Stabilizer::calcTorque (const hrp::Matrix33& rot)
{
  m_robot->calcForwardKinematics(true, true);
  // buffers for the unit vector method
  hrp::Vector3 root_w_x_v;
  hrp::Vector3 g(0, 0, 9.80665);
  root_w_x_v = m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p));
  m_robot->rootLink()->dvo = g - root_w_x_v;   // dv = g, dw = 0
  m_robot->rootLink()->dw.setZero();

  hrp::Vector3 root_f;
  hrp::Vector3 root_t;
  m_robot->calcInverseDynamics(m_robot->rootLink(), root_f, root_t);
  // if (loop % 200 == 0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   std::cerr << ":cog "; rats::print_vector(std::cerr, m_robot->calcCM());
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->link(i)->name << " "
  //               << m_robot->joint(i)->jointId << " "
  //               << m_robot->link(i)->m << " ";
  //     hrp::Vector3 tmpc = m_robot->link(i)->p + m_robot->link(i)->R * m_robot->link(i)->c;
  //     rats::print_vector(std::cerr, tmpc, false);
  //     std::cerr << " ";
  //     rats::print_vector(std::cerr, m_robot->link(i)->c, false);
  //     std::cerr << ")" << std::endl;
  //   }
  // }
  // if (loop % 200 == 0) {
  //   std::cerr << ":IV1 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
  hrp::dmatrix contact_mat, contact_mat_inv;
  std::vector<hrp::Vector3> contact_p;
  for (size_t j = 0; j < 2; j++) contact_p.push_back(m_robot->sensor<hrp::ForceSensor>(stikp[j].sensor_name)->link->p);
  calcContactMatrix(contact_mat, contact_p);
  hrp::calcSRInverse(contact_mat, contact_mat_inv, 0.0);
  hrp::dvector root_ft(6);
  for (size_t j = 0; j < 3; j++) root_ft(j) = root_f(j);
  for (size_t j = 0; j < 3; j++) root_ft(j+3) = root_t(j);
  hrp::dvector contact_ft(2*6);
  contact_ft = contact_mat_inv * root_ft;
  // if (loop%200==0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   // std::cerr << ":ftv "; rats::print_vector(std::cerr, ftv);
  //   // std::cerr << ":aa "; rats::print_matrix(std::cerr, aa);
  //   // std::cerr << ":dv "; rats::print_vector(std::cerr, dv);
  // }
  // for (size_t j = 0; j < 2; j++) {//numContacts
  // hrp::JointPathEx jm = hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor<hrp::ForceSensor>(stikp[j].sensor_name)->link, dt);
  if ( control_mode == MODE_ST ) {
      for (size_t j = 0; j < stikp.size(); j++) {
          STIKParam& ikp = stikp[j];
          hrp::Link* target = m_robot->link(ikp.target_name);
          size_t idx = contact_states_index_map[ikp.ee_name];
          const hrp::JointPath jm(m_robot->rootLink(), target);
          hrp::dmatrix JJ(6, jm.numJoints());
          jm.calcJacobian(JJ);
          hrp::dvector ft(6);
          // for (size_t i = 0; i < 6; i++) ft(i) = contact_ft(i+j*6);
          ft.segment(0,3) = rot * ikp.ref_force;
          ft.segment(3,3) = rot * (ikp.ref_moment - ikp.localp.cross(ikp.ref_force));
          hrp::dvector tq_from_extft(jm.numJoints());
          tq_from_extft = JJ.transpose() * ft;
          // if (loop%200==0) {
          //   std::cerr << ":ft "; rats::print_vector(std::cerr, ft);
          //   std::cerr << ":JJ "; rats::print_matrix(std::cerr, JJ);
          //   std::cerr << ":tq_from_extft "; rats::print_vector(std::cerr, tq_from_extft);
          // }
          for (size_t i = 0; i < jm.numJoints(); i++) jm.joint(i)->u -= tq_from_extft(i);
      }
  }
  //hrp::dmatrix MM(6,m_robot->numJoints());
  //m_robot->calcMassMatrix(MM);
  // if (loop % 200 == 0) {
  //   std::cerr << ":INVDYN2 (list "; rats::print_vector(std::cerr, root_f, false);
  //   std::cerr << " "; rats::print_vector(std::cerr, root_t, false);
  //   std::cerr << ")" << std::endl;
  //   // hrp::dvector tqv(m_robot->numJoints());
  //   // for(int i = 0; i < m_robot->numJoints(); ++i){p
  //   //   tqv[m_robot->joint(i)->jointId] = m_robot->joint(i)->u;
  //   // }
  //   // std::cerr << ":IV2 "; rats::print_vector(std::cerr, tqv);
  //   std::cerr << ":IV2 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
};

void Stabilizer::calcRUNST() {
  if ( m_robot->numJoints() == qRef.size() ) {
    std::vector<std::string> target_name;
    target_name.push_back("L_ANKLE_R");
    target_name.push_back("R_ANKLE_R");

    double angvelx_ref;// = (m_rpyRef.data.r - pangx_ref)/dt;
    double angvely_ref;// = (m_rpyRef.data.p - pangy_ref)/dt;
    //pangx_ref = m_rpyRef.data.r;
    //pangy_ref = m_rpyRef.data.p;
    double angvelx = (rpy(0) - pangx)/dt;
    double angvely = (rpy(1) - pangy)/dt;
    pangx = rpy(0);
    pangy = rpy(1);

    // update internal robot model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      qorg[i] = m_robot->joint(i)->q;
      m_robot->joint(i)->q = qRef[i];
      qrefv[i] = qRef[i];
    }
    //double orgjq = m_robot->link("L_FOOT")->joint->q;
    double orgjq = m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q;
    //set root
    m_robot->rootLink()->p = hrp::Vector3(0,0,0);
    //m_robot->rootLink()->R = hrp::rotFromRpy(m_rpyRef.data.r,m_rpyRef.data.p,m_rpyRef.data.y);
    m_robot->calcForwardKinematics();
    hrp::Vector3 target_root_p = m_robot->rootLink()->p;
    hrp::Matrix33 target_root_R = m_robot->rootLink()->R;
    hrp::Vector3 target_foot_p[2];
    hrp::Matrix33 target_foot_R[2];
    for (size_t i = 0; i < 2; i++) {
      target_foot_p[i] = m_robot->link(target_name[i])->p;
      target_foot_R[i] = m_robot->link(target_name[i])->R;
    }
    hrp::Vector3 target_fm = (m_robot->link(target_name[0])->p + m_robot->link(target_name[1])->p)/2;
    //hrp::Vector3 org_cm = m_robot->rootLink()->R.transpose() * (m_robot->calcCM() - m_robot->rootLink()->p);
    hrp::Vector3 org_cm = m_robot->rootLink()->R.transpose() * (target_fm - m_robot->rootLink()->p);

    // stabilizer loop
    if ( ( wrenches[1].size() > 0 && wrenches[0].size() > 0 )
         //( m_wrenches[ST_LEFT].data[2] > m_robot->totalMass()/4 || m_wrenches[ST_RIGHT].data[2] > m_robot->totalMass()/4 )
         ) {

      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = qorg[i];
      }
      // set root
      double rddx;// = k_run_b[0] * (m_rpyRef.data.r - m_rpy.data.r) + d_run_b[0] * (angvelx_ref - angvelx);
      double rddy;// = k_run_b[1] * (m_rpyRef.data.p - m_rpy.data.p) + d_run_b[1] * (angvely_ref - angvely);
      rdx += rddx * dt;
      rx += rdx * dt;
      rdy += rddy * dt;
      ry += rdy * dt;
      //rx += rddx * dt;
      //ry += rddy * dt;
      // if (DEBUGP2) {
      //   std::cerr << "REFRPY " <<  m_rpyRef.data.r << " " << m_rpyRef.data.p << std::endl;
      // }
      // if (DEBUGP2) {
      //   std::cerr << "RPY " <<  m_rpy.data.r << " " << m_rpy.data.p << std::endl;
      //   std::cerr << " rx " << rx << " " << rdx << " " << rddx << std::endl;
      //   std::cerr << " ry " << ry << " " << rdy << " " << rddy << std::endl;
      // }
      hrp::Vector3 root_p_s;
      hrp::Matrix33 root_R_s;
      rats::rotm3times(root_R_s, hrp::rotFromRpy(rx, ry, 0), target_root_R);
      if (DEBUGP2) {
        hrp::Vector3 tmp = hrp::rpyFromRot(root_R_s);
        std::cerr << "RPY2 " <<  tmp(0) << " " << tmp(1) << std::endl;
      }
      root_p_s = target_root_p + target_root_R * org_cm - root_R_s * org_cm;
      //m_robot->calcForwardKinematics();
      // FK
      m_robot->rootLink()->R = root_R_s;
      m_robot->rootLink()->p = root_p_s;
      if (DEBUGP2) {
        std::cerr << " rp " << root_p_s[0] << " " << root_p_s[1] << " " << root_p_s[2] << std::endl;
      }
      m_robot->calcForwardKinematics();
      //
      hrp::Vector3 current_fm = (m_robot->link(target_name[0])->p + m_robot->link(target_name[1])->p)/2;

      // 3D-LIP model contorller
      hrp::Vector3 dr = target_fm - current_fm;
      //hrp::Vector3 dr = current_fm - target_fm ;
      hrp::Vector3 dr_vel = (dr - pdr)/dt;
      pdr = dr;
      double tau_y = - m_torque_k[0] * dr(0) - m_torque_d[0] * dr_vel(0);
      double tau_x = m_torque_k[1] * dr(1) + m_torque_d[1] * dr_vel(1);
      if (DEBUGP2) {
        dr*=1e3;
        dr_vel*=1e3;
        std::cerr << "dr " << dr(0) << " " << dr(1) << " " << dr_vel(0) << " " << dr_vel(1) << std::endl;
        std::cerr << "tau_x " << tau_x << std::endl;
        std::cerr << "tau_y " << tau_y << std::endl;
      }

      double gamma = 0.5; // temp
      double tau_xl[2];
      double tau_yl[2];
      double xfront = 0.125;
      double xrear = 0.1;
      double yin = 0.02;
      double yout = 0.15;
      double mg = m_robot->totalMass() * 9.8 * 0.9;// margin
      double tq_y_ulimit = mg * xrear;
      double tq_y_llimit = -1 * mg * xfront;
      double tq_x_ulimit = mg * yout;
      double tq_x_llimit = mg * yin;
      // left
      tau_xl[0] = gamma * tau_x;
      tau_yl[0] = gamma * tau_y;
      tau_xl[0] = vlimit(tau_xl[0], tq_x_llimit, tq_x_ulimit);
      tau_yl[0] = vlimit(tau_yl[0], tq_y_llimit, tq_y_ulimit);
      // right
      tau_xl[1]= (1- gamma) * tau_x;
      tau_yl[1]= (1- gamma) * tau_y;
      tau_xl[1] = vlimit(tau_xl[1], -1*tq_x_ulimit, -1*tq_x_llimit);
      tau_yl[1] = vlimit(tau_yl[1], tq_y_llimit, tq_y_ulimit);

      double dleg_x[2];
      double dleg_y[2];
      double tau_y_total = (wrenches[1][4] + wrenches[0][4]) / 2;
      double dpz;
      if (DEBUGP2) {
        std::cerr << "tq limit " << tq_x_ulimit << " " << tq_x_llimit << " " << tq_y_ulimit << " " << tq_y_llimit << std::endl;
      }
      for (size_t i = 0; i < 2; i++) {
        // dleg_x[i] = m_tau_x[i].update(m_wrenches[i].data[3], tau_xl[i]);
        // dleg_y[i] = m_tau_y[i].update(m_wrenches[i].data[4], tau_yl[i]);
        //dleg_x[i] = m_tau_x[i].update(m_wrenches[i].data[3], tau_xl[i]);
        dleg_x[i] = m_tau_x[i].update(0,0);
        dleg_y[i] = m_tau_y[i].update(tau_y_total, tau_yl[i]);
        if (DEBUGP2) {
          std::cerr << i << " dleg_x " << dleg_x[i] << std::endl;
          std::cerr << i << " dleg_y " << dleg_y[i] << std::endl;
          std::cerr << i << " t_x " << wrenches[i][3] << " "<< tau_xl[i] << std::endl;
          std::cerr << i << " t_y " << wrenches[i][4] << " "<< tau_yl[i] << std::endl;
        }
      }

      // calc leg rot
      hrp::Matrix33 target_R[2];
      hrp::Vector3 target_p[2];
      for (size_t i = 0; i < 2; i++) {
        //rats::rotm3times(target_R[i], hrp::rotFromRpy(dleg_x[i], dleg_y[i], 0), target_foot_R[i]);
        rats::rotm3times(target_R[i], hrp::rotFromRpy(0, dleg_y[i], 0), target_foot_R[i]);
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        target_p[i] = target_foot_p[i];
      }
      // 1=>left, 2=>right
      double refdfz = 0;
      dpz = m_f_z.update((wrenches[0][2] - wrenches[1][2]), refdfz);
      //target_p[0](2) = target_foot_p[0](2) + dpz/2;
      //target_p[1](2) = target_foot_p[1](2) - dpz/2;
      target_p[0](2) = target_foot_p[0](2);
      target_p[1](2) = target_foot_p[1](2);

      // IK
      for (size_t i = 0; i < 2; i++) {
        hrp::Link* target = m_robot->link(target_name[i]);
        hrp::Vector3 vel_p, vel_r;
        vel_p = target_p[i] - target->p;
        rats::difference_rotation(vel_r, target->R, target_R[i]);
        //jpe_v[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, DEBUGP);
        //jpe_v[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, false);
        //m_robot->joint(m_robot->link(target_name[i])->jointId)->q = dleg_y[i] + orgjq;
      }
      // m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq + m_rpy.data.p;
      // m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq + m_rpy.data.p;
      m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq;
      m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq;
    } else {
      // reinitialize
      for (int i = 0; i < 2; i++) {
        m_tau_x[i].reset();
        m_tau_y[i].reset();
        m_f_z.reset();
      }
    }
  }
}

void Stabilizer::setSwingSupportJointServoGains()
{
    static double tmp_landing2support_transition_time = landing2support_transition_time;
    for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        hrp::JointPathExPtr jpe = jpe_v[i];
        if (ikp.contact_phase == SWING_PHASE && !ref_contact_states[i] && controlSwingSupportTime[i] < swing2landing_transition_time+landing_phase_time) { // SWING -> LANDING
            ikp.contact_phase = LANDING_PHASE;
            ikp.phase_time = 0;
            for(size_t j = 0; j < ikp.support_pgain.size(); j++) {
                m_robotHardwareService0->setServoPGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.landing_pgain(j),swing2landing_transition_time);
                m_robotHardwareService0->setServoDGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.landing_dgain(j),swing2landing_transition_time);
            }
        }
        if (ikp.contact_phase == LANDING_PHASE && act_contact_states[i] && ref_contact_states[i] && ikp.phase_time > swing2landing_transition_time) { // LANDING -> SUPPORT
            ikp.contact_phase = SUPPORT_PHASE;
            ikp.phase_time = 0;
            tmp_landing2support_transition_time = std::min(landing2support_transition_time, controlSwingSupportTime[i]);
            for(size_t j = 0; j < ikp.support_pgain.size(); j++) {
                m_robotHardwareService0->setServoPGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.support_pgain(j),tmp_landing2support_transition_time);
                m_robotHardwareService0->setServoDGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.support_dgain(j),tmp_landing2support_transition_time);
            }
        }
        // if (ikp.contact_phase == SUPPORT_PHASE && !act_contact_states[i] && ikp.phase_time > tmp_landing2support_transition_time) { // SUPPORT -> SWING
        if (ikp.contact_phase == SUPPORT_PHASE && !act_contact_states[i] && ikp.phase_time > tmp_landing2support_transition_time+support_phase_min_time
            && ( (ref_contact_states[i] && controlSwingSupportTime[i] < 0.2) || !ref_contact_states[i] )) { // SUPPORT -> SWING
            ikp.contact_phase = SWING_PHASE;
            ikp.phase_time = 0;
            for(size_t j = 0; j < ikp.support_pgain.size(); j++) {
                m_robotHardwareService0->setServoPGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.swing_pgain(j),support2swing_transition_time);
                m_robotHardwareService0->setServoDGainPercentageWithTime(jpe->joint(j)->name.c_str(),ikp.swing_dgain(j),support2swing_transition_time);
            }
        }
        ikp.phase_time += dt;
    }
}

void Stabilizer::calcExternalForce(const hrp::Vector3& cog, const hrp::Vector3& zmp, const hrp::Matrix33& rot)
{
    // cog, zmp must be in the same coords with stikp.ref_forece
    hrp::Vector3 total_force = hrp::Vector3::Zero();
    for (size_t j = 0; j < stikp.size(); j++) {
        total_force(2) += stikp[j].ref_force(2); // only fz
    }
    total_force.segment(0,2) = (cog.segment(0,2) - zmp.segment(0,2))*total_force(2)/(cog(2) - zmp(2)); // overwrite fxy
    for (size_t j = 0; j < stikp.size(); j++) {
        STIKParam& ikp = stikp[j];
        if (on_ground && total_force(2) > 1e-6) ikp.ref_force.segment(0,2) += rot.transpose() * total_force.segment(0,2) * ikp.ref_force(2)/total_force(2);// set fx,fy
    }
}

void Stabilizer::moveBasePosRotForBodyRPYControl ()
{
  // Body rpy control
  //   Basically Equation (1) and (2) in the paper [1]
  hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(target_root_R);
  bool is_root_rot_limit = false;
  for (size_t i = 0; i < 2; i++) {
    d_rpy[i] = transition_smooth_gain * (eefm_body_attitude_control_gain[i] * (ref_root_rpy(i) - act_base_rpy(i)) - 1/eefm_body_attitude_control_time_const[i] * d_rpy[i]) * dt + d_rpy[i];
    d_rpy[i] = vlimit(d_rpy[i], -1 * root_rot_compensation_limit[i], root_rot_compensation_limit[i]);
    is_root_rot_limit = is_root_rot_limit || (std::fabs(std::fabs(d_rpy[i]) - root_rot_compensation_limit[i] ) < 1e-5); // near the limit
  }
  rats::rotm3times(current_root_R, target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
  m_robot->rootLink()->R = current_root_R;
  m_robot->rootLink()->p = target_root_p + target_root_R * rel_cog - current_root_R * rel_cog;
  m_robot->calcForwardKinematics();
  current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
  current_base_pos = m_robot->rootLink()->p;
  if ( DEBUGP || (is_root_rot_limit && loop%200==0) ) {
    std::cerr << "[" << print_str << "] Root rot control" << std::endl;
    if (is_root_rot_limit) std::cerr << "[" << print_str << "]   Root rot limit reached!!" << std::endl;
    std::cerr << "[" << print_str << "]   ref = [" << rad2deg(ref_root_rpy(0)) << " " << rad2deg(ref_root_rpy(1)) << "], "
              << "act = [" << rad2deg(act_base_rpy(0)) << " " << rad2deg(act_base_rpy(1)) << "], "
              << "cur = [" << rad2deg(current_base_rpy(0)) << " " << rad2deg(current_base_rpy(1)) << "], "
              << "limit = [" << rad2deg(root_rot_compensation_limit[0]) << " " << rad2deg(root_rot_compensation_limit[1]) << "][deg]" << std::endl;
  }
};

double Stabilizer::vlimit(double value, double llimit_value, double ulimit_value)
{
  if (value > ulimit_value) {
    return ulimit_value;
  } else if (value < llimit_value) {
    return llimit_value;
  }
  return value;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, double llimit_value, double ulimit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
    if (value(i) > ulimit_value) {
      ret(i) = ulimit_value;
    } else if (value(i) < llimit_value) {
      ret(i) = llimit_value;
    } else {
      ret(i) = value(i);
    }
  }
  return ret;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, const hrp::Vector3& limit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
    if (value(i) > limit_value(i)) {
      ret(i) = limit_value(i);
    } else if (value(i) < -1 * limit_value(i)) {
      ret(i) = -1 * limit_value(i);
    } else {
      ret(i) = value(i);
    }
  }
  return ret;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, const hrp::Vector3& llimit_value, const hrp::Vector3& ulimit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
    if (value(i) > ulimit_value(i)) {
      ret(i) = ulimit_value(i);
    } else if (value(i) < llimit_value(i)) {
      ret(i) = llimit_value(i);
    } else {
      ret(i) = value(i);
    }
  }
  return ret;
}

void Stabilizer::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
  rats::coordinates leg_c[2], tmpc;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  for (size_t i = 0; i < stikp.size(); i++) {
    if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
    hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name)->link;
    leg_c[i].pos = target->p + target->R * foot_origin_offset[i];
    hrp::Vector3 xv1(target->R * ex);
    xv1(2)=0.0;
    xv1.normalize();
    hrp::Vector3 yv1(ez.cross(xv1));
    leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
    leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
    leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);
  }
  if (ref_contact_states[contact_states_index_map["rleg"]] &&
      ref_contact_states[contact_states_index_map["lleg"]]) {
    rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
    foot_origin_pos = tmpc.pos;
    foot_origin_rot = tmpc.rot;
  } else if (ref_contact_states[contact_states_index_map["rleg"]]) {
    foot_origin_pos = leg_c[contact_states_index_map["rleg"]].pos;
    foot_origin_rot = leg_c[contact_states_index_map["rleg"]].rot;
  } else {
    foot_origin_pos = leg_c[contact_states_index_map["lleg"]].pos;
    foot_origin_rot = leg_c[contact_states_index_map["lleg"]].rot;
  }
}

bool Stabilizer::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z)
{
  double tmpzmpx = 0;
  double tmpzmpy = 0;
  double tmpfz = 0, tmpfz2 = 0.0;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (!is_zmp_calc_enable[i]) continue;
    hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
    hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
    hrp::Matrix33 tmpR;
    rats::rotm3times(tmpR, sensor->link->R, sensor->localR);
    hrp::Vector3 nf = tmpR * hrp::Vector3(wrenches[i][0], wrenches[i][1], wrenches[i][2]);
    hrp::Vector3 nm = tmpR * hrp::Vector3(wrenches[i][3], wrenches[i][4], wrenches[i][5]);
    tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
    tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
    tmpfz += nf(2);
    // calc ee-local COP
    hrp::Link* target = m_robot->link(stikp[i].target_name);
    hrp::Matrix33 eeR = target->R * stikp[i].localR;
    hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * stikp[i].localp)); // ee-local force sensor pos
    nf = eeR.transpose() * nf;
    nm = eeR.transpose() * nm;
    // ee-local total moment and total force at ee position
    double tmpcopmy = nf(2) * ee_fsp(0) - nf(0) * ee_fsp(2) - nm(1);
    double tmpcopmx = nf(2) * ee_fsp(1) - nf(1) * ee_fsp(2) + nm(0);
    double tmpcopfz = nf(2);
    copInfo[i*3] = tmpcopmx;
    copInfo[i*3+1] = tmpcopmy;
    copInfo[i*3+2] = tmpcopfz;
    prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
    tmpfz2 += prev_act_force_z[i];
  }
  if (tmpfz2 < contact_decision_threshold) {
    ret_zmp = act_zmp;
    return false; // in the air
  } else {
    ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
    return true; // on ground
  }
};

void Stabilizer::calcStateForEmergencySignal()
{
  // COP Check
  bool is_cop_outside = false;
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] Check Emergency State (seq = " << (is_seq_interpolating?"interpolating":"empty") << ")" << std::endl;
  }
  if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
    if (DEBUGP) {
      std::cerr << "[" << print_str << "] COP check" << std::endl;
    }
    for (size_t i = 0; i < stikp.size(); i++) {
      if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
      // check COP inside
      if (copInfo[i*3+2] > 20.0 ) {
        hrp::Vector3 tmpcop(copInfo[i*3+1]/copInfo[i*3+2], copInfo[i*3]/copInfo[i*3+2], 0);
        is_cop_outside = is_cop_outside ||
          (!szd->is_inside_foot(tmpcop, stikp[i].ee_name=="lleg", cop_check_margin) ||
           szd->is_front_of_foot(tmpcop, cop_check_margin) ||
           szd->is_rear_of_foot(tmpcop, cop_check_margin));
        if (DEBUGP) {
          std::cerr << "[" << print_str << "]   [" << stikp[i].ee_name << "] "
                    << "outside(" << !szd->is_inside_foot(tmpcop, stikp[i].ee_name=="lleg", cop_check_margin) << ") "
                    << "front(" << szd->is_front_of_foot(tmpcop, cop_check_margin) << ") "
                    << "rear(" << szd->is_rear_of_foot(tmpcop, cop_check_margin) << ")" << std::endl;
        }
      } else {
        is_cop_outside = true;
      }
    }
  } else {
    is_cop_outside = false;
  }
  // CP Check
  bool is_cp_outside = false;
  is_emergency_motion = false;
  if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
    Eigen::Vector2d tmp_cp = act_cp.head(2);
    std::vector<bool> tmp_cs(2,true);
    hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(target_root_R);
    // stop manipulation
    if (!is_walking) {
      szd->get_margined_vertices(margined_support_polygon_vetices);
      szd->calc_convex_hull(margined_support_polygon_vetices, act_contact_states, rel_ee_pos, rel_ee_rot);
      is_cp_outside = !szd->is_inside_support_polygon(tmp_cp, - sbp_cog_offset);
    }
    // start emregency stepping
    szd->get_vertices(support_polygon_vetices);
    szd->calc_convex_hull(support_polygon_vetices, act_contact_states, rel_ee_pos, rel_ee_rot);
    is_emergency_step = !szd->is_inside_support_polygon(tmp_cp, - sbp_cog_offset);
    // // comment in if you use emergency motion
    // if (!is_walking && falling_direction != 0) {
    //   if (((falling_direction == 1 || falling_direction == 2) && std::fabs(rad2deg(ref_root_rpy(1))) > 0.5) ||
    //       ((falling_direction == 3 || falling_direction == 4) && std::fabs(rad2deg(ref_root_rpy(0))) > 0.5))
    //     is_emergency_motion = true;
    // }
    if (DEBUGP) {
      std::cerr << "[" << print_str << "] CP value " << "[" << act_cp(0) << "," << act_cp(1) << "] [m], "
                << "sbp cog offset [" << sbp_cog_offset(0) << " " << sbp_cog_offset(1) << "], outside ? "
                << (is_cp_outside?"Outside":"Inside")
                << std::endl;
    }
    if (is_cp_outside) {
      if (initial_cp_too_large_error || loop % static_cast <int>(0.2/dt) == 0 ) { // once per 0.2[s]
        std::cerr << "[" << print_str << "] ["
                  << "] CP too large error " << "[" << act_cp(0) << "," << act_cp(1) << "] [m]" << std::endl;
      }
      initial_cp_too_large_error = false;
    } else {
      initial_cp_too_large_error = true;
    }
  }
  // tilt Check // cannot use
  hrp::Vector3 fall_direction = hrp::Vector3::Zero();
  bool is_falling = false, will_fall = false;
  // {
  //   double total_force = 0.0;
  //   for (size_t i = 0; i < stikp.size(); i++) {
  //     if (is_zmp_calc_enable[i]) {
  //       if (is_walking) {
  //         if (projected_normal.at(i).norm() > sin(tilt_margin[0])) {
  //           will_fall = true;
  //           if (m_will_fall_counter[i] % static_cast <int>(1.0/dt) == 0 ) { // once per 1.0[s]
  //             std::cerr << "[" << print_str << "] ["
  //                       << "] " << stikp[i].ee_name << " cannot support total weight, "
  //                       << "swgsuptime : " << controlSwingSupportTime[i] << ", state : " << ref_contact_states[i]
  //                       << ", otherwise robot will fall down toward " << "(" << projected_normal.at(i)(0) << "," << projected_normal.at(i)(1) << ") direction" << std::endl;
  //           }
  //           m_will_fall_counter[i]++;
  //         } else {
  //           m_will_fall_counter[i] = 0;
  //         }
  //       }
  //       fall_direction += projected_normal.at(i) * act_force.at(i).norm();
  //       total_force += act_force.at(i).norm();
  //     }
  //   }
  //   if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
  //     fall_direction = fall_direction / total_force;
  //   } else {
  //     fall_direction = hrp::Vector3::Zero();
  //   }
  //   if (fall_direction.norm() > sin(tilt_margin[1])) {
  //     is_falling = true;
  //     if (m_is_falling_counter % static_cast <int>(0.2/dt) == 0) { // once per 0.2[s]
  //       std::cerr << "[" << print_str << "] ["
  //                 << "] robot is falling down toward " << "(" << fall_direction(0) << "," << fall_direction(1) << ") direction" << std::endl;
  //     }
  //     m_is_falling_counter++;
  //   } else {
  //     m_is_falling_counter = 0;
  //   }
  // }
  // Total check for emergency signal
  switch (emergency_check_mode) {
  case OpenHRP::AutoBalancerService::NO_CHECK:
    is_emergency = false;
    break;
  case OpenHRP::AutoBalancerService::COP:
    is_emergency = is_cop_outside && is_seq_interpolating;
    break;
  case OpenHRP::AutoBalancerService::CP:
    is_emergency = is_cp_outside;
    break;
  case OpenHRP::AutoBalancerService::TILT:
    is_emergency = will_fall || is_falling;
    break;
  default:
    break;
  }
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] EmergencyCheck ("
              << (emergency_check_mode == OpenHRP::AutoBalancerService::NO_CHECK?"NO_CHECK": (emergency_check_mode == OpenHRP::AutoBalancerService::COP?"COP":"CP") )
              << ") " << (is_emergency?"emergency":"non-emergency") << std::endl;
  }
  rel_ee_pos.clear();
  rel_ee_rot.clear();
  rel_ee_name.clear();
  rel_ee_rot_for_ik.clear();
};

void Stabilizer::calcSwingSupportLimbGain ()
{
  for (size_t i = 0; i < stikp.size(); i++) {
    STIKParam& ikp = stikp[i];
    if (ref_contact_states[i]) { // Support
      // Limit too large support time increment. Max time is 3600.0[s] = 1[h], this assumes that robot's one step time is smaller than 1[h].
      ikp.support_time = std::min(3600.0, ikp.support_time+dt);
      // In some PC, does not work because the first line is optimized out.
      // ikp.support_time += dt;
      // ikp.support_time = std::min(3600.0, ikp.support_time);
      if (ikp.support_time > eefm_pos_transition_time) {
        ikp.swing_support_gain = (controlSwingSupportTime[i] / eefm_pos_transition_time);
      } else {
        ikp.swing_support_gain = (ikp.support_time / eefm_pos_transition_time);
      }
      ikp.swing_support_gain = std::max(0.0, std::min(1.0, ikp.swing_support_gain));
    } else { // Swing
      ikp.swing_support_gain = 0.0;
      ikp.support_time = 0.0;
    }
  }
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] SwingSupportLimbGain = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << stikp[i].swing_support_gain << " ";
    std::cerr << "], ref_contact_states = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << ref_contact_states[i] << " ";
    std::cerr << "], sstime = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << controlSwingSupportTime[i] << " ";
    std::cerr << "], toeheel_ratio = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << toeheel_ratio[i] << " ";
    std::cerr << "], support_time = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << stikp[i].support_time << " ";
    std::cerr << "]" << std::endl;
  }
}

void Stabilizer::calcTPCC() {
  // stabilizer loop
  // Choi's feedback law
  hrp::Vector3 cog = m_robot->calcCM();
  hrp::Vector3 newcog = hrp::Vector3::Zero();
  hrp::Vector3 dcog(ref_cog - act_cog);
  hrp::Vector3 dzmp(ref_zmp - act_zmp);
  for (size_t i = 0; i < 2; i++) {
    double uu = ref_cogvel(i) - k_tpcc_p[i] * transition_smooth_gain * dzmp(i)
      + k_tpcc_x[i] * transition_smooth_gain * dcog(i);
    newcog(i) = uu * dt + cog(i);
  }

  moveBasePosRotForBodyRPYControl ();

  // target at ee => target at link-origin
  hrp::Vector3 target_link_p[stikp.size()];
  hrp::Matrix33 target_link_R[stikp.size()];
  for (size_t i = 0; i < stikp.size(); i++) {
    rats::rotm3times(target_link_R[i], target_ee_R[i], stikp[i].localR.transpose());
    target_link_p[i] = target_ee_p[i] - target_ee_R[i] * stikp[i].localCOPPos;
  }
  // solveIK
  //   IK target is link origin pos and rot, not ee pos and rot.
  //for (size_t jj = 0; jj < 5; jj++) {
  size_t max_ik_loop_count = 0;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (max_ik_loop_count < stikp[i].ik_loop_count) max_ik_loop_count = stikp[i].ik_loop_count;
  }
  for (size_t jj = 0; jj < max_ik_loop_count; jj++) {
    hrp::Vector3 tmpcm = m_robot->calcCM();
    for (size_t i = 0; i < 2; i++) {
      m_robot->rootLink()->p(i) = m_robot->rootLink()->p(i) + 0.9 * (newcog(i) - tmpcm(i));
    }
    m_robot->calcForwardKinematics();
    for (size_t i = 0; i < stikp.size(); i++) {
      if (is_ik_enable[i]) {
        jpe_v[i]->calcInverseKinematics2Loop(target_link_p[i], target_link_R[i], 1.0, stikp[i].avoid_gain, stikp[i].reference_gain, &qrefv, transition_smooth_gain);
      }
    }
  }
}

void Stabilizer::calcEEForceMomentControl()
{
  // stabilizer loop
  // return to referencea
  m_robot->rootLink()->R = target_root_R;
  m_robot->rootLink()->p = target_root_p;
  for ( int i = 0; i < m_robot->numJoints(); i++ ) {
    m_robot->joint(i)->q = qrefv[i];
  }
  for (size_t i = 0; i < jpe_v.size(); i++) {
    if (is_ik_enable[i]) {
      for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
        int idx = jpe_v[i]->joint(j)->jointId;
        m_robot->joint(idx)->q = qorg[idx];
      }
    }
  }
  // Fix for toe joint
  for (size_t i = 0; i < jpe_v.size(); i++) {
    if (is_ik_enable[i]) {
      if (jpe_v[i]->numJoints() == 7) {
        int idx = jpe_v[i]->joint(jpe_v[i]->numJoints() -1)->jointId;
        m_robot->joint(idx)->q = qrefv[idx];
      }
    }
  }

  // State calculation for swing ee compensation
  //   joint angle : current control output
  //   root pos : target root p
  //   root rot : actual root rot
  {
    // Calc status
    m_robot->rootLink()->p = target_root_p;
    m_robot->rootLink()->R = act_root_R;
    m_robot->calcForwardKinematics();
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
    // Calculate foot_origin_coords-relative ee pos and rot
    // Subtract them from target_ee_diff_xx
    for (size_t i = 0; i < stikp.size(); i++) {
      hrp::Link* target = m_robot->link(stikp[i].target_name);
      stikp[i].target_ee_diff_p -= foot_origin_rot.transpose() * (target->p + target->R * stikp[i].localp - foot_origin_pos);
      stikp[i].act_theta = Eigen::AngleAxisd(foot_origin_rot.transpose() * target->R * stikp[i].localR);
    }
  }

  // State calculation for control : calculate "current" state
  //   joint angle : current control output
  //   root pos : target + keep COG against rpy control
  //   root rot : target + rpy control
  moveBasePosRotForBodyRPYControl ();

  // Convert d_foot_pos in foot origin frame => "current" world frame
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  calcFootOriginCoords (foot_origin_pos, foot_origin_rot);

  // Swing ee compensation.
  if (use_act_states) calcSwingEEModification();

  // solveIK
  //   IK target is link origin pos and rot, not ee pos and rot.
  std::vector<hrp::Vector3> tmpp(stikp.size());
  std::vector<hrp::Matrix33> tmpR(stikp.size());
  double tmp_d_pos_z_root = 0.0;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (is_ik_enable[i]) {
      // Add damping_control compensation to target value
      if (is_feedback_control_enable[i]) {
        rats::rotm3times(tmpR[i], target_ee_R[i], hrp::rotFromRpy(-1*stikp[i].ee_d_foot_rpy));
        // foot force difference control version
        // total_target_foot_p[i](2) = target_foot_p[i](2) + (i==0?0.5:-0.5)*zctrl;
        // foot force independent damping control
        tmpp[i] = target_ee_p[i] - (foot_origin_rot * stikp[i].d_foot_pos);
      } else {
        tmpp[i] = target_ee_p[i];
        tmpR[i] = target_ee_R[i];
      }
      // Add swing ee compensation
      rats::rotm3times(tmpR[i], tmpR[i], hrp::rotFromRpy(rel_ee_rot_for_ik[i].transpose() * stikp[i].d_rpy_swing));
      tmpp[i] = tmpp[i] + (foot_origin_rot * stikp[i].d_pos_swing);
    }
  }

  limbStretchAvoidanceControl(tmpp ,tmpR);

  // IK
  for (size_t i = 0; i < stikp.size(); i++) {
    if (is_ik_enable[i]) {
      for (size_t jj = 0; jj < stikp[i].ik_loop_count; jj++) {
        jpe_v[i]->calcInverseKinematics2Loop(tmpp[i], tmpR[i], 1.0, 0.001, 0.01, &qrefv, transition_smooth_gain,
                                             //stikp[i].localCOPPos;
                                             stikp[i].localp,
                                             stikp[i].localR);
      }
    }
  }
  prev_ref_foot_origin_rot = foot_origin_rot;
}

// Swing ee compensation.
//   Calculate compensation values to minimize the difference between "current" foot-origin-coords-relative pos and rot and "target" foot-origin-coords-relative pos and rot for swing ee.
//   Input  : target_ee_diff_p, ref_theta, act_theta
//   Output : d_pos_swing, d_rpy_swing
void Stabilizer::calcSwingEEModification ()
{
  for (size_t i = 0; i < stikp.size(); i++) {
    // Calc compensation values
    double limit_pos = 50 * 1e-3; // 50[mm] limit
    double limit_rot = deg2rad(30); // 30[deg] limit
    if (ref_contact_states != prev_ref_contact_states) {
      stikp[i].d_pos_swing = (ref_foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * stikp[i].d_pos_swing;
      stikp[i].d_rpy_swing = (ref_foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * stikp[i].d_rpy_swing;
      stikp[i].prev_d_pos_swing = (ref_foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * stikp[i].prev_d_pos_swing;
      stikp[i].prev_d_rpy_swing = (ref_foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * stikp[i].prev_d_rpy_swing;
    }
    if (ref_contact_states[contact_states_index_map[stikp[i].ee_name]] || act_contact_states[contact_states_index_map[stikp[i].ee_name]]) {
      // If actual contact or target contact is ON, do not use swing ee compensation. Exponential zero retrieving.
      if (!is_foot_touch[i] && is_walking) {
        double tmp_ratio = 1.0;
        swing_modification_interpolator[stikp[i].ee_name]->clear();
        swing_modification_interpolator[stikp[i].ee_name]->set(&tmp_ratio);
        tmp_ratio = 0.0;
        swing_modification_interpolator[stikp[i].ee_name]->setGoal(&tmp_ratio, stikp[i].touchoff_remain_time, true);
        is_foot_touch[i] = true;
      }
    } else if (swing_modification_interpolator[stikp[i].ee_name]->isEmpty()) {
      /* position */
      {
        hrp::Vector3 tmpdiffp = stikp[i].eefm_swing_pos_spring_gain.cwiseProduct(stikp[i].target_ee_diff_p) * dt + stikp[i].d_pos_swing;
        double lvlimit = -50 * 1e-3 * dt, uvlimit = 50 * 1e-3 * dt; // 50 [mm/s]
        hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_pos_swing + lvlimit * hrp::Vector3::Ones();
        hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_pos_swing + uvlimit * hrp::Vector3::Ones();
        stikp[i].d_pos_swing = vlimit(vlimit(tmpdiffp, -1 * limit_pos, limit_pos), limit_by_lvlimit, limit_by_uvlimit);
      }
      /* rotation */
      {
        Eigen::AngleAxisd prev_omega = stikp[i].omega;
        stikp[i].omega = (stikp[i].act_theta.inverse() * stikp[i].ref_theta);
        stikp[i].omega.angle() *= stikp[i].eefm_swing_rot_spring_gain(0) * dt;
        stikp[i].omega = stikp[i].omega * prev_omega;
        hrp::Vector3 tmpdiffr = hrp::rpyFromRot(stikp[i].omega.toRotationMatrix());
        double lvlimit = deg2rad(-40.0*dt), uvlimit = deg2rad(40.0*dt); // 20 [deg/s]
        hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_rpy_swing + lvlimit * hrp::Vector3::Ones();
        hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_rpy_swing + uvlimit * hrp::Vector3::Ones();
        stikp[i].d_rpy_swing = vlimit(vlimit(tmpdiffr, -1 * limit_rot, limit_rot), limit_by_lvlimit, limit_by_uvlimit);
      }
      is_foot_touch[i] = false;
      touchdown_d_pos[i] = stikp[i].d_pos_swing;
      touchdown_d_rpy[i] = stikp[i].d_rpy_swing;
    }
    if (!swing_modification_interpolator[stikp[i].ee_name]->isEmpty()) {
      double tmp_ratio = 0.0;
      swing_modification_interpolator[stikp[i].ee_name]->setGoal(&tmp_ratio, stikp[i].touchoff_remain_time, true);
      swing_modification_interpolator[stikp[i].ee_name]->get(&tmp_ratio, true);
      stikp[i].d_pos_swing = touchdown_d_pos[i] * tmp_ratio;
      stikp[i].d_rpy_swing = touchdown_d_rpy[i] * tmp_ratio;
    }
    stikp[i].prev_d_pos_swing = stikp[i].d_pos_swing;
    stikp[i].prev_d_rpy_swing = stikp[i].d_rpy_swing;
  }
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] Swing foot control" << std::endl;
    for (size_t i = 0; i < stikp.size(); i++) {
      std::cerr << "[" << print_str << "]   "
                << "d_rpy_swing (" << stikp[i].ee_name << ")  = " << (stikp[i].d_rpy_swing / M_PI * 180.0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[deg], "
                << "d_pos_swing (" << stikp[i].ee_name << ")  = " << (stikp[i].d_pos_swing * 1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm]" << std::endl;
    }
  }
};

void Stabilizer::limbStretchAvoidanceControl (const std::vector<hrp::Vector3>& ee_p, const std::vector<hrp::Matrix33>& ee_R)
{
  double tmp_d_pos_z_root = 0.0, prev_d_pos_z_root = d_pos_z_root;
  if (use_limb_stretch_avoidance) {
    for (size_t i = 0; i < stikp.size(); i++) {
      if (is_ik_enable[i]) {
        // Check whether inside limb length limitation
        hrp::Link* parent_link = m_robot->link(stikp[i].parent_name);
        hrp::Vector3 targetp = (ee_p[i] - ee_R[i] * stikp[i].localR.transpose() * stikp[i].localp) - parent_link->p; // position from parent to target link (world frame)
        double limb_length_limitation = stikp[i].max_limb_length - stikp[i].limb_length_margin;
        double tmp = limb_length_limitation * limb_length_limitation - targetp(0) * targetp(0) - targetp(1) * targetp(1);
        if (targetp.norm() > limb_length_limitation && tmp >= 0) {
          tmp_d_pos_z_root = std::min(tmp_d_pos_z_root, targetp(2) + std::sqrt(tmp));
        }
      }
    }
    // Change root link height depending on limb length
    d_pos_z_root = tmp_d_pos_z_root == 0.0 ? calcDampingControl(d_pos_z_root, limb_stretch_avoidance_time_const) : tmp_d_pos_z_root;
  } else {
    d_pos_z_root = calcDampingControl(d_pos_z_root, limb_stretch_avoidance_time_const);
  }
  d_pos_z_root = vlimit(d_pos_z_root, prev_d_pos_z_root + limb_stretch_avoidance_vlimit[0], prev_d_pos_z_root + limb_stretch_avoidance_vlimit[1]);
  m_robot->rootLink()->p(2) += d_pos_z_root;
}

void Stabilizer::setBoolSequenceParam (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name)
{
  std::vector<bool> prev_values;
  prev_values.resize(st_bool_values.size());
  copy (st_bool_values.begin(), st_bool_values.end(), prev_values.begin());
  if (st_bool_values.size() != output_bool_values.length()) {
    std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set. Length " << st_bool_values.size() << " != " << output_bool_values.length() << std::endl;
  } else if ( (control_mode != MODE_IDLE) ) {
    std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set. Current control_mode is " << control_mode << std::endl;
  } else {
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      st_bool_values[i] = output_bool_values[i];
    }
  }
  std::cerr << "[" << print_str << "]   " << prop_name << " is ";
  for (size_t i = 0; i < st_bool_values.size(); i++) {
    std::cerr <<"[" << st_bool_values[i] << "]";
  }
  std::cerr << "(set = ";
  for (size_t i = 0; i < output_bool_values.length(); i++) {
    std::cerr <<"[" << output_bool_values[i] << "]";
  }
  std::cerr << ", prev = ";
  for (size_t i = 0; i < prev_values.size(); i++) {
    std::cerr <<"[" << prev_values[i] << "]";
  }
  std::cerr << ")" << std::endl;
};

void Stabilizer::setBoolSequenceParamWithCheckContact (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name)
{
  std::vector<bool> prev_values;
  prev_values.resize(st_bool_values.size());
  copy (st_bool_values.begin(), st_bool_values.end(), prev_values.begin());
  if (st_bool_values.size() != output_bool_values.length()) {
    std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set. Length " << st_bool_values.size() << " != " << output_bool_values.length() << std::endl;
  } else if ( control_mode == MODE_IDLE ) {
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      st_bool_values[i] = output_bool_values[i];
    }
  } else {
    std::vector<size_t> failed_indices;
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      if ( (st_bool_values[i] != output_bool_values[i]) ) { // If mode change
        if (!ref_contact_states[i] ) { // reference contact_states should be OFF
          st_bool_values[i] = output_bool_values[i];
        } else {
          failed_indices.push_back(i);
        }
      }
    }
    if (failed_indices.size() > 0) {
      std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set partially. failed_indices is [";
      for (size_t i = 0; i < failed_indices.size(); i++) {
        std::cerr << failed_indices[i] << " ";
      }
      std::cerr << "]" << std::endl;
    }
  }
  std::cerr << "[" << print_str << "]   " << prop_name << " is ";
  for (size_t i = 0; i < st_bool_values.size(); i++) {
    std::cerr <<"[" << st_bool_values[i] << "]";
  }
  std::cerr << "(set = ";
  for (size_t i = 0; i < output_bool_values.length(); i++) {
    std::cerr <<"[" << output_bool_values[i] << "]";
  }
  std::cerr << ", prev = ";
  for (size_t i = 0; i < prev_values.size(); i++) {
    std::cerr <<"[" << prev_values[i] << "]";
  }
  std::cerr << ")" << std::endl;
};

std::string Stabilizer::getStabilizerAlgorithmString (OpenHRP::AutoBalancerService::STAlgorithm _st_algorithm)
{
  switch (_st_algorithm) {
  case OpenHRP::AutoBalancerService::TPCC:
    return "TPCC";
  case OpenHRP::AutoBalancerService::EEFM:
    return "EEFM";
  case OpenHRP::AutoBalancerService::EEFMQP:
    return "EEFMQP";
  case OpenHRP::AutoBalancerService::EEFMQPCOP:
    return "EEFMQPCOP";
  case OpenHRP::AutoBalancerService::EEFMQPCOP2:
    return "EEFMQPCOP2";
  default:
    return "";
  }
};

void Stabilizer::calcDiffFootOriginExtMoment ()
{
  // calc reference ext moment around foot origin pos
  // static const double grav = 9.80665; /* [m/s^2] */
  double mg = total_mass * eefm_gravitational_acceleration;
  hrp::Vector3 ref_ext_moment = hrp::Vector3(mg * ref_cog(1) - ref_total_foot_origin_moment(0),
                                             -mg * ref_cog(0) - ref_total_foot_origin_moment(1),
                                             0);
  // calc act ext moment around foot origin pos
  hrp::Vector3 act_ext_moment = hrp::Vector3(mg * act_cog(1) - act_total_foot_origin_moment(0),
                                             -mg * act_cog(0) - act_total_foot_origin_moment(1),
                                             0);
  // Do not calculate actual value if in the air, because of invalid act_zmp.
  if ( !on_ground ) act_ext_moment = ref_ext_moment;
  // Calc diff
  diff_foot_origin_ext_moment = ref_ext_moment - act_ext_moment;
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] DiffStaticBalancePointOffset" << std::endl;
    std::cerr << "[" << print_str << "]   "
              << "ref_ext_moment = " << ref_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm], "
              << "act_ext_moment = " << act_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm], "
              << "diff ext_moment = " << diff_foot_origin_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm]" << std::endl;
  }
};
