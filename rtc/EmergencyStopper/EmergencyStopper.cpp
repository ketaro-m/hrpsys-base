// -*- C++ -*-
/*!
 * @file  EmergencyStopper.cpp
 * @brief emergency stopper
 * $Date$
 *
 * $Id$
 */

#include "hrpsys/util/VectorConvert.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <math.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "hrpsys/idl/RobotHardwareService.hh"

#include "EmergencyStopper.h"
#include <iomanip>

typedef coil::Guard<coil::Mutex> Guard;

#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif

// Module specification
// <rtc-template block="module_spec">
static const char* emergencystopper_spec[] =
{
    "implementation_id", "EmergencyStopper",
    "type_name",         "EmergencyStopper",
    "description",       "emergency stopper",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
};
// </rtc-template>

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

EmergencyStopper::EmergencyStopper(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_qEmergencyIn("qEmergency", m_qEmergency),
      m_emergencySignalIn("emergencySignal", m_emergencySignal),
      m_emergencyFallMotionIn("emergencyFallMotion", m_emergencyFallMotion),
      m_servoStateIn("servoStateIn", m_servoState),
      m_qOut("q", m_q),
      m_qTouchWallOut("qTouchWall", m_qTouchWall),
      m_emergencyModeOut("emergencyMode", m_emergencyMode),
      m_beepCommandOut("beepCommand", m_beepCommand),
      m_touchWallMotionSolvedOut("touchWallMotionSolved", m_touchWallMotionSolved),
      m_EmergencyStopperServicePort("EmergencyStopperService"),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      loop(0),
      emergency_stopper_beep_count(0),
      dummy(0)
{
    m_service0.emergencystopper(this);
}

EmergencyStopper::~EmergencyStopper()
{
}


#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t EmergencyStopper::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("qEmergency", m_qEmergencyIn);
    addInPort("emergencySignal", m_emergencySignalIn);
    addInPort("emergencyFallMotion", m_emergencyFallMotionIn);
    addInPort("servoStateIn", m_servoStateIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("qTouchWall", m_qTouchWallOut);
    addOutPort("emergencyMode", m_emergencyModeOut);
    addOutPort("beepCommand", m_beepCommandOut);
    addOutPort("touchWallMotionSolved", m_touchWallMotionSolvedOut);

    // Set service provider to Ports
    m_EmergencyStopperServicePort.registerProvider("service0", "EmergencyStopperService", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_EmergencyStopperServicePort);

    // </rtc-template>

    // Setup robot model
    RTC::Properties& prop = getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());
    m_robot = hrp::BodyPtr(new hrp::Body());

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    OpenHRP::BodyInfo_var binfo;
    binfo = hrp::loadBodyInfo(prop["model"].c_str(),
                              CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    if (CORBA::is_nil(binfo)) {
        std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]"
                  << std::endl;
        return RTC::RTC_ERROR;
    }
    if (!loadBodyFromBodyInfo(m_robot, binfo)) {
        std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
        return RTC::RTC_ERROR;
    }

    // Setting for wrench data ports (real + virtual)
    OpenHRP::LinkInfoSequence_var lis = binfo->links();
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    for ( unsigned int k = 0; k < lis->length(); k++ ) {
        OpenHRP::SensorInfoSequence& sensors = lis[k].sensors;
        for ( unsigned int l = 0; l < sensors.length(); l++ ) {
            if ( std::string(sensors[l].type) == "Force" ) {
                fsensor_names.push_back(std::string(sensors[l].name));
            }
        }
    }
    unsigned int npforce = fsensor_names.size();
    //   find names for virtual force sensors
    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    unsigned int nvforce = virtual_force_sensor.size()/10;
    for (unsigned int i=0; i<nvforce; i++){
        fsensor_names.push_back(virtual_force_sensor[i*10+0]);
    }
    //   add ports for all force sensors
    unsigned int nforce  = npforce + nvforce;
    m_wrenchesRef.resize(nforce);
    m_wrenches.resize(nforce);
    m_wrenchesIn.resize(nforce);
    m_wrenchesOut.resize(nforce);
    for (unsigned int i=0; i<nforce; i++){
        m_wrenchesIn[i] = new InPort<TimedDoubleSeq>(std::string(fsensor_names[i]+"In").c_str(), m_wrenchesRef[i]);
        m_wrenchesOut[i] = new OutPort<TimedDoubleSeq>(std::string(fsensor_names[i]+"Out").c_str(), m_wrenches[i]);
        m_wrenchesRef[i].data.length(6);
        m_wrenchesRef[i].data[0] = m_wrenchesRef[i].data[1] = m_wrenchesRef[i].data[2] = 0.0;
        m_wrenchesRef[i].data[3] = m_wrenchesRef[i].data[4] = m_wrenchesRef[i].data[5] = 0.0;
        m_wrenches[i].data.length(6);
        m_wrenches[i].data[0] = m_wrenches[i].data[1] = m_wrenches[i].data[2] = 0.0;
        m_wrenches[i].data[3] = m_wrenches[i].data[4] = m_wrenches[i].data[5] = 0.0;
        registerInPort(std::string(fsensor_names[i]+"In").c_str(), *m_wrenchesIn[i]);
        registerOutPort(std::string(fsensor_names[i]+"Out").c_str(), *m_wrenchesOut[i]);
    }

    // initialize member variables
    is_stop_mode = prev_is_stop_mode = false;
    is_emergency_fall_motion = false;
    is_initialized = false;
    emergency_mode = 0;

    recover_time = retrieve_time = 0;
    recover_time_dt = 1.0;
    default_recover_time = 2.5/m_dt;
    default_retrieve_time = 1;
    default_retrieve_duration = 1;
    solved = false;
    //default_retrieve_time = 1.0/m_dt;
    m_stop_posture.resize(m_robot->numJoints(), 0.0);
    m_motion_posture.resize(m_robot->numJoints(), 0.0);
    m_stop_wrenches = new double[nforce*6];
    m_tmp_wrenches = new double[nforce*6];
    m_interpolator = new interpolator(m_robot->numJoints(), recover_time_dt);
    m_interpolator->setName(std::string(m_profile.instance_name)+" interpolator");
    m_wrenches_interpolator = new interpolator(nforce*6, recover_time_dt);
    m_wrenches_interpolator->setName(std::string(m_profile.instance_name)+" interpolator wrenches");

    m_q.data.length(m_robot->numJoints());
    for(unsigned int i=0; i<m_robot->numJoints(); i++){
        m_q.data[i] = 0;
    }
    m_qTouchWall.data.length(m_robot->numJoints());
    for(unsigned int i=0; i<nforce; i++){
        for(int j=0; j<6; j++){
            m_wrenches[i].data[j] = 0;
            m_stop_wrenches[i*6+j] = 0;
        }
    }

    m_servoState.data.length(m_robot->numJoints());
    for(unsigned int i = 0; i < m_robot->numJoints(); i++) {
        m_servoState.data[i].length(1);
        int status = 0;
        status |= 1<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
        status |= 1<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
        status |= 1<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
        status |= 0<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
        status |= 0<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
        m_servoState.data[i][0] = status;
    }

    emergency_stopper_beep_freq = static_cast<int>(1.0/(2.0*m_dt)); // 2 times / 1[s]
    m_beepCommand.data.length(bc.get_num_beep_info());
    return RTC::RTC_OK;
}




RTC::ReturnCode_t EmergencyStopper::onFinalize()
{
    delete m_interpolator;
    delete m_wrenches_interpolator;
    delete m_stop_wrenches;
    delete m_tmp_wrenches;
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t EmergencyStopper::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t EmergencyStopper::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t EmergencyStopper::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    Guard guard(m_mutex);
    if (is_stop_mode) {
        is_stop_mode = false;
        emergency_mode = 0;
        is_emergency_fall_motion = false;
        solved = false;
        recover_time = 0;
        m_interpolator->setGoal(m_qRef.data.get_buffer(), m_dt);
        m_interpolator->get(m_q.data.get_buffer());
    }
    return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper::onExecute(RTC::UniqueId ec_id)
{
    unsigned int numJoints = m_robot->numJoints();
    loop++;
    if (m_servoStateIn.isNew()) {
        m_servoStateIn.read();
    }
    if (!is_initialized) {
        if (m_qRefIn.isNew()) {
            m_qRefIn.read();
            is_initialized = true;
        } else {
            return RTC::RTC_OK;
        }
    }

    // read data
    if (m_qRefIn.isNew()) {
        // joint angle
        m_qRefIn.read();
        assert(m_qRef.data.length() == numJoints);
        std::vector<double> current_posture;
        for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ) {
            current_posture.push_back(m_qRef.data[i]);
        }
        m_input_posture_queue.push(current_posture);
        while ((int)m_input_posture_queue.size() > default_retrieve_time) {
            m_input_posture_queue.pop();
        }
        if (!is_stop_mode) {
            for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ) {
                if (recover_time > 0) { // Until releasing is finished, do not use m_stop_posture in input queue because too large error.
                    m_stop_posture[i] = m_q.data[i];
                } else {
                  m_stop_posture[i] = m_input_posture_queue.front()[i];
                }
            }
        }
        {
            // double tmpq[] = {-6.827925e-08, -3.735005e-06, -0.929562, 2.46032, -1.53045, 3.839724e-06, 6.827917e-08, 3.735005e-06, -0.929564, 2.46032, -1.53045, -3.839724e-06}; // CHIDORI
            double tmpq[] = {7.164374e-09, -1.013475e-07, -1.00249, 1.6208, -0.618309, 1.068601e-07, -3.914983e-10, 2.927592e-08, -0.82775, 1.6555, -0.827748, -3.091128e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.39626, -0.349066, -0.087266, 0.0, 0.0, 0.0, -0.349066, 0.0, -1.39626, 0.349066, 0.087266, 0.0, 0.0, 0.0, -0.349066, -1.39626, 1.39626, -1.39626, 1.39626}; // JAXON
            // double tmpq[] = {
            //   // -0.031978, -0.302196, 0.052625, 0.623091, -0.052851, -0.319949, -0.000000, -0.000291, -0.032075, -0.302298, 0.052653, 0.623232, -0.052929, -0.320630, -0.000000, -0.000063, 0.261282, 0.869966, -0.087213, -0.000439, 0.000020, -0.000080, 0.000013, 0.890118, 0.261531, -0.086439, 0.087310, -0.523310, -0.000004, 0.000032, -0.000003, 0.890118, 0.298616, 0.298616, 0.259056, 0.294777, 0.294777, 0.298616, 0.298616, 0.259056, 0.294777, 0.294777, 0.311048, 0.311048, 0.221701, 0.221701 // stretch larm
            //     0.000005, -0.348759, 0.000012, 0.630766, 0.000030, -0.278725, 0.000000, -0.000211, -0.000005, -0.348793, 0.000024, 0.630764, 0.000044, -0.278784, -0.000000, -0.000065, 0.261541, 0.086439, -0.087310, -0.523306, 0.000004, 0.000033, 0.000003, 0.890118, 0.261541, -0.086438, 0.087310, -0.523306, -0.000004, 0.000033, -0.000003, 0.890118, 0.298616, 0.298616, 0.259056, 0.294777, 0.294777, 0.298616, 0.298616, 0.259056, 0.294777, 0.294777, 0.311048, 0.311048, 0.221701, 0.221701 // reset-pose
            // }; // RHP4B
            for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ) {
                if (recover_time > 0 && !is_stop_mode) { // Until releasing is finished, do not use m_stop_posture in input queue because too large error.
                    m_motion_posture[i] = m_q.data[i];
                } else {
                  if (!solved) {
                    if (sizeof(tmpq)/sizeof(double) == m_qRef.data.length()) m_motion_posture[i] = tmpq[i];
                    else m_motion_posture[i] = m_q.data[i];
                  } else m_motion_posture[i] = m_qEmergency.data[i];
                }
            }
        }
        // wrench
        for ( size_t i = 0; i < m_wrenchesIn.size(); i++ ) {
            if ( m_wrenchesIn[i]->isNew() ) {
                m_wrenchesIn[i]->read();
            }
        }
        std::vector<double> current_wrench;
        for ( unsigned int i= 0; i < m_wrenchesRef.size(); i++ ) {
            for (int j = 0; j < 6; j++ ) {
                current_wrench.push_back(m_wrenchesRef[i].data[j]);
            }
        }
        m_input_wrenches_queue.push(current_wrench);
        while ((int)m_input_wrenches_queue.size() > default_retrieve_time) {
            m_input_wrenches_queue.pop();
        }
        if (!is_stop_mode) {
            for ( unsigned int i= 0; i < m_wrenchesRef.size(); i++ ) {
                for (int j = 0; j < 6; j++ ) {
                    if (recover_time > 0) {
                        m_stop_wrenches[i*6+j] = m_wrenches[i].data[j];
                    } else {
                        m_stop_wrenches[i*6+j] = m_input_wrenches_queue.front()[i*6+j];
                    }
                }
            }
        }
    }

    if (m_emergencySignalIn.isNew()){
        m_emergencySignalIn.read();
        emergency_mode = m_emergencySignal.data;
        if ( emergency_mode == 0 ) {
          if (is_stop_mode) {
            Guard guard(m_mutex);
            std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                      << "] emergencySignal is reset!" << std::endl;
            is_stop_mode = false;
          }
        } else if (!is_stop_mode) {
          Guard guard(m_mutex);
          switch (emergency_mode) {
          case 1:
            std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                      << "] emergencySignal is set!" << std::endl;
            is_stop_mode = true;
            break;
          case 2:
            std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                      << "] emergencyTouchWall is set!" << std::endl;
            is_stop_mode = true;
            break;
          default:
            break;
          }
        }
    }
    if (m_emergencyFallMotionIn.isNew()) {
        m_emergencyFallMotionIn.read();
        if ( !m_emergencyFallMotion.data ) {
            Guard guard(m_mutex);
            std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                      << "] emergencyFallMotion is reset!" << std::endl;
            is_stop_mode = false;
            emergency_mode = 0;
            is_emergency_fall_motion = false;
        } else {
            Guard guard(m_mutex);
            std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                      << "] emergencyFallMotion is set!" << std::endl;
            is_stop_mode = true;
            is_emergency_fall_motion = true;
        }
    }
    if (m_qEmergencyIn.isNew()) {
      m_qEmergencyIn.read();
    }
    if (is_stop_mode && !prev_is_stop_mode) {
        retrieve_time = default_retrieve_duration;
        // Reflect current output joint angles to interpolator state
        m_interpolator->set(m_q.data.get_buffer());
        get_wrenches_array_from_data(m_wrenches, m_tmp_wrenches);
        m_wrenches_interpolator->set(m_tmp_wrenches);
    }

    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] is_stop_mode : " << is_stop_mode << " recover_time : "  << recover_time << "[s] retrieve_time : " << retrieve_time << "[s]" << std::endl;
    }

    //     mode : is_stop_mode : recover_time  : set as q
    // release  :        false :            0  : qRef
    // recover  :        false :         >  0  : q'
    // stop     :         true :  do not care  : q(do nothing)
    if (!is_stop_mode) {
        if (recover_time > 0) {
            recover_time = recover_time - recover_time_dt;
            m_interpolator->setGoal(m_qRef.data.get_buffer(), recover_time);
            m_interpolator->get(m_q.data.get_buffer());
            get_wrenches_array_from_data(m_wrenchesRef, m_tmp_wrenches);
            m_wrenches_interpolator->setGoal(m_tmp_wrenches, recover_time);
            m_wrenches_interpolator->get(m_tmp_wrenches);
            set_wrenches_data_from_array(m_wrenches, m_tmp_wrenches);
        } else {
            for ( unsigned int i = 0; i < m_q.data.length(); i++ ) {
                m_q.data[i] = m_qRef.data[i];
            }
            for ( unsigned int i = 0; i < m_wrenches.size(); i++ ) {
                for ( int j = 0; j < 6; j++ ) {
                    m_wrenches[i].data[j] = m_wrenchesRef[i].data[j];
                }
            }
        }
    } else { // stop mode
        recover_time = default_recover_time;
        if (retrieve_time > 0 ) {
            retrieve_time = retrieve_time - recover_time_dt;
            if (is_emergency_fall_motion) m_interpolator->setGoal(m_motion_posture.data(), retrieve_time);
            else m_interpolator->setGoal(m_stop_posture.data(), retrieve_time);
            m_interpolator->get(m_q.data.get_buffer());
            m_wrenches_interpolator->setGoal(m_stop_wrenches, retrieve_time);
            m_wrenches_interpolator->get(m_tmp_wrenches);
            set_wrenches_data_from_array(m_wrenches, m_tmp_wrenches);
        } else {
            // Do nothing
        }
    }

    // write data
    if (DEBUGP) {
        std::cerr << "q: ";
        for (unsigned int i = 0; i < numJoints; i++) {
            std::cerr << " " << m_q.data[i] ;
        }
        std::cerr << std::endl;
        std::cerr << "wrenches: ";
        for (unsigned int i = 0; i < m_wrenches.size(); i++) {
            for (int j = 0; j < 6; j++ ) {
                std::cerr << " " << m_wrenches[i].data[j];
            }
        }
        std::cerr << std::endl;
    }
    m_q.tm = m_qRef.tm;
    m_qOut.write();
    for (size_t i = 0; i < m_wrenches.size(); i++) {
      m_wrenches[i].tm = m_qRef.tm;
    }
    for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
      m_wrenchesOut[i]->write();
    }

    m_emergencyMode.data = emergency_mode;
    m_emergencyMode.tm = m_qRef.tm;
    m_emergencyModeOut.write();
    m_touchWallMotionSolved.data = solved;
    m_touchWallMotionSolved.tm = m_qRef.tm;
    m_touchWallMotionSolvedOut.write();

    prev_is_stop_mode = is_stop_mode;

    // beep sound for emergency stop alert
    //  check servo for emergency stop beep sound
    bool has_servoOn = false;
    for (unsigned int i = 0; i < m_robot->numJoints(); i++ ){
        int servo_state = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
        has_servoOn = has_servoOn || (servo_state == 1);
    }
    //  beep
    if ( is_stop_mode && has_servoOn ) { // If stop mode and some joint is servoOn
      if ( emergency_stopper_beep_count % emergency_stopper_beep_freq == 0 && emergency_stopper_beep_count % (emergency_stopper_beep_freq * 3) != 0 ) {
        bc.startBeep(2352, emergency_stopper_beep_freq*0.7);
      } else {
        bc.stopBeep();
      }
        emergency_stopper_beep_count++;
    } else {
        emergency_stopper_beep_count = 0;
        bc.stopBeep();
    }
    bc.setDataPort(m_beepCommand);
    m_beepCommand.tm = m_qRef.tm;
    if (bc.isWritable()) m_beepCommandOut.write();
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t EmergencyStopper::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t EmergencyStopper::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t EmergencyStopper::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t EmergencyStopper::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t EmergencyStopper::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

bool EmergencyStopper::stopMotion()
{
    Guard guard(m_mutex);
    if (!is_stop_mode) {
        is_stop_mode = true;
        std::cerr << "[" << m_profile.instance_name << "] stopMotion is called" << std::endl;
    }
    return true;
}

bool EmergencyStopper::releaseMotion()
{
    Guard guard(m_mutex);
    {
        is_stop_mode = false;
        emergency_mode = 0;
        is_emergency_fall_motion = false;
        solved = false;
        std::cerr << "[" << m_profile.instance_name << "] releaseMotion is called" << std::endl;
    }
    return true;
}

bool EmergencyStopper::getEmergencyStopperParam(OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] getEmergencyStopperParam" << std::endl;
    i_param.default_recover_time = default_recover_time*m_dt;
    i_param.default_retrieve_time = default_retrieve_time*m_dt;
    i_param.default_retrieve_duration = default_retrieve_duration*m_dt;    
    i_param.is_stop_mode = is_stop_mode;
    return true;
};

bool EmergencyStopper::setEmergencyStopperParam(const OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] setEmergencyStopperParam" << std::endl;
    default_recover_time = i_param.default_recover_time/m_dt;
    default_retrieve_time = i_param.default_retrieve_time/m_dt;
    default_retrieve_duration = i_param.default_retrieve_duration/m_dt;
    std::cerr << "[" << m_profile.instance_name << "]   default_recover_time = " << default_recover_time*m_dt << "[s], default_retrieve_time = " << default_retrieve_time*m_dt << "[s], default_retrieve_duration = " << default_retrieve_duration*m_dt << "[s]" << std::endl;
    return true;
};

bool EmergencyStopper::setEmergencyJointAngles(const double *angles, const bool _solved)
{
  // interpolate in Autobalancer
  for (size_t i = 0; i < m_robot->numJoints(); i++) {
    // tmp for jaxon choreonoid
    if (i == 33 || i == 35) m_stop_posture[i] = -1.39626;
    else if (i == 34 || i == 36) m_stop_posture[i] = 1.39626;
    else m_stop_posture[i] = deg2rad(angles[i]);
    m_qTouchWall.data[i] = m_stop_posture[i];
  }
  retrieve_time = default_retrieve_time;
  m_qTouchWall.tm = m_qRef.tm;
  m_qTouchWallOut.write();
  solved = _solved;
  is_stop_mode = true;
  return true;
};

extern "C"
{

    void EmergencyStopperInit(RTC::Manager* manager)
    {
        RTC::Properties profile(emergencystopper_spec);
        manager->registerFactory(profile,
                                 RTC::Create<EmergencyStopper>,
                                 RTC::Delete<EmergencyStopper>);
    }

};


