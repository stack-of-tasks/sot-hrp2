/*
 * Copyright 2011,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of HRP2Controller.
 * HRP2Controller is not a free software,
 * it contains information related to HRP-2 which involves
 * that you either purchased the proper license to have access to
 * those informations, or that you signed the appropriate
 * Non-Disclosure agreement.
 *
 *
 */

#include <fstream>
#include <map>

#include <sot/core/debug.hh>

#include "sot-hrp2-device.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace std;

const double SoTHRP2Device::TIMESTEP_DEFAULT = 0.005;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SoTHRP2Device,"Device");

SoTHRP2Device::SoTHRP2Device(std::string RobotName):
  dgsot::Device(RobotName),
  timestep_(TIMESTEP_DEFAULT),
  previousState_ (),
  robotState_ ("StackOfTasks(" + RobotName + ")::output(vector)::robotState"),
  accelerometerSOUT_
  ("StackOfTasks(" + RobotName + ")::output(vector)::accelerometer"),
  gyrometerSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::gyrometer"),
  currentSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::currents"),
  p_gainsSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::p_gains"),
  d_gainsSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::d_gains"),
  mlforces (6),
  pose (),
  accelerometer_ (3),
  gyrometer_ (3),
  torques_(),
  baseff_ ()
{
  sotDEBUGIN(25) ;
  for( int i=0;i<4;++i ) { withForceSignals[i] = true; }
  signalRegistration (robotState_ << accelerometerSOUT_ << gyrometerSOUT_
                      << currentSOUT_ << p_gainsSOUT_ << d_gainsSOUT_);
  dg::Vector data (3); data.setZero ();
  accelerometerSOUT_.setConstant (data);
  gyrometerSOUT_.setConstant (data);
  baseff_.resize(7);

  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  docstring =
      "\n"
      "    Integrate dynamics for time step provided as input\n"
      "\n"
      "      take one floating point number as input\n"
      "\n";
  addCommand("increment",
             makeCommandVoid1((Device&)*this,
                              &Device::increment, docstring));
  
  sotDEBUGOUT(25);
}

SoTHRP2Device::~SoTHRP2Device()
{ }

void SoTHRP2Device::setSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  sotDEBUGIN(25) ;
  map<string,dgsot::SensorValues>::iterator it;
  int t = stateSOUT.getTime () + 1;

  it = SensorsIn.find("forces");
  if (it!=SensorsIn.end())
  {

    // Implements force recollection.
    const vector<double>& forcesIn = it->second.getValues();
    for(int i=0;i<4;++i)
    {
      for(int j=0;j<6;++j)
        mlforces(j) = forcesIn[i*6+j];
      forcesSOUT[i]->setConstant(mlforces);
      forcesSOUT[i]->setTime (t);
    }
  }

  it = SensorsIn.find("attitude");
  bool attitudeSensorFound_ = false;
  if (it!=SensorsIn.end())
  {
    attitudeSensorFound_ = true;
    const vector<double>& attitude = it->second.getValues ();
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 3; ++j)
        pose (i, j) = attitude [i * 3 + j];
    attitudeSOUT.setConstant (pose);
    attitudeSOUT.setTime (t);
  }

  it = SensorsIn.find("base_position");
  bool ffPositionSensorFound_ = false;
  Eigen::Vector3d base_p;
  if (it!=SensorsIn.end())
  {
    ffPositionSensorFound_ = true;
    const vector<double>& baseposition = it->second.getValues ();
    base_p << baseposition[0], baseposition[1], baseposition[2];
  }

  it = SensorsIn.find("joints");
  if (it!=SensorsIn.end())
  {
    const vector<double>& anglesIn = it->second.getValues();
    mlRobotState.resize (anglesIn.size () + 6);
    if (ffPositionSensorFound_ == true && attitudeSensorFound_ == true) {
      mlRobotState.head<3>() = base_p;
      mlRobotState.segment<3>(3) = pose.eulerAngles(2,1,0).reverse();
    }
    else
      mlRobotState.head<6>().setZero();

    for (unsigned i = 0; i < anglesIn.size(); ++i)
      mlRobotState (i + 6) = anglesIn[i];
    robotState_.setConstant(mlRobotState);
    robotState_.setTime(t);
  }

  it = SensorsIn.find("accelerometer_0");
  if (it!=SensorsIn.end())
  {
    const vector<double>& accelerometer =
        SensorsIn ["accelerometer_0"].getValues ();
    for (std::size_t i=0; i<3; ++i)
      accelerometer_ (i) = accelerometer [i];
    accelerometerSOUT_.setConstant (accelerometer_);
    accelerometerSOUT_.setTime (t);
  }

  it = SensorsIn.find("gyrometer_0");
  if (it!=SensorsIn.end())
  {
    const vector<double>& gyrometer = SensorsIn ["gyrometer_0"].getValues ();
    for (std::size_t i=0; i<3; ++i)
      gyrometer_ (i) = gyrometer [i];
    gyrometerSOUT_.setConstant (gyrometer_);
    gyrometerSOUT_.setTime (t);
  }

  it = SensorsIn.find("torques");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& torques = SensorsIn["torques"].getValues();
    torques_.resize(torques.size());
    for(std::size_t i = 0; i < torques.size(); ++i)
      torques_ (i) = torques [i];
    pseudoTorqueSOUT.setConstant(torques_);
    pseudoTorqueSOUT.setTime(t);
  }
  
  it = SensorsIn.find("currents");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& currents = SensorsIn["currents"].getValues();
    currents_.resize(currents.size());
    for(std::size_t i = 0; i < currents.size(); ++i)
      currents_ (i) = currents[i];
    currentSOUT_.setConstant(currents_);
    currentSOUT_.setTime(t);
  }

  it = SensorsIn.find("p_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& p_gains = SensorsIn["p_gains"].getValues();
    p_gains_.resize(p_gains.size());
    for(std::size_t i = 0; i < p_gains.size(); ++i)
      p_gains_ (i) = p_gains[i];
    p_gainsSOUT_.setConstant(p_gains_);
    p_gainsSOUT_.setTime(t);
  }

  it = SensorsIn.find("d_gains");
  if (it!=SensorsIn.end())
  {
    const std::vector<double>& d_gains = SensorsIn["d_gains"].getValues();
    d_gains_.resize(d_gains.size());
    for(std::size_t i = 0; i < d_gains.size(); ++i)
      d_gains_ (i) = d_gains[i];
    d_gainsSOUT_.setConstant(d_gains_);
    d_gainsSOUT_.setTime(t);
  }

  sotDEBUGOUT(25);
}

void SoTHRP2Device::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTHRP2Device::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTHRP2Device::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTHRP2Device::getControl(map<string,dgsot::ControlValues> &controlOut)
{
  sotDEBUGIN(25) ;
  vector<double> anglesOut;
  anglesOut.resize(state_.size());
  
  // Integrate control
  increment(timestep_);
  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << ((previousState_.size() == state_.size())?
                                    (state_ - previousState_) : state_ ) << std::endl;
  previousState_ = state_;

  // Specify the joint values for the controller.
  if (anglesOut.size()!=state_.size()-6)
    anglesOut.resize(state_.size()-6);

  for(unsigned int i=6; i < state_.size();++i)
    anglesOut[i-6] = state_(i);
  controlOut["joints"].setValues(anglesOut);

  // Read zmp reference from input signal if plugged
  int time = controlSIN.getTime ();
  zmpSIN.recompute (time + 1);
  // Express ZMP in free flyer reference frame
  dg::Vector zmpGlobal (4);
  for (unsigned int i = 0; i < 3; ++i)
    zmpGlobal(i) = zmpSIN(time + 1)(i);
  zmpGlobal(3) = 1.;
  dgsot::MatrixHomogeneous inversePose;
  inversePose = freeFlyerPose().inverse(Eigen::Affine);
  dg::Vector localZmp(4); localZmp = inversePose.matrix() * zmpGlobal;
  vector<double> ZMPRef(3);
  for(unsigned int i=0;i<3;++i)
    ZMPRef[i] = localZmp(i);

  controlOut["zmp"].setName("zmp");
  controlOut["zmp"].setValues(ZMPRef);

  // Update position of freeflyer in global frame
  Eigen::Vector3d transq_(freeFlyerPose().translation());
  dg::sot::VectorQuaternion qt_(freeFlyerPose().linear());

  //translation
  for(int i=0; i<3; i++) baseff_[i] = transq_(i);
  
  //rotation: quaternion
  baseff_[3] = qt_.w();
  baseff_[4] = qt_.x();
  baseff_[5] = qt_.y();
  baseff_[6] = qt_.z();

  controlOut["baseff"].setValues(baseff_);
  sotDEBUGOUT(25) ;
}
