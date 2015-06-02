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
  mlforces (6),
  pose (),
  accelerometer_ (3),
  gyrometer_ (3),
  torques_(),
  baseff_ ()
{
  sotDEBUGIN(25) ;
  for( int i=0;i<4;++i ) { withForceSignals[i] = true; }
  signalRegistration (robotState_ << accelerometerSOUT_ << gyrometerSOUT_);
  ml::Vector data (3); data.setZero ();
  accelerometerSOUT_.setConstant (data);
  gyrometerSOUT_.setConstant (data);
  baseff_.resize(12);

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
  if (it!=SensorsIn.end())
  {
    const vector<double>& attitude = it->second.getValues ();
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 3; ++j)
        pose (i, j) = attitude [i * 3 + j];
    attitudeSOUT.setConstant (pose);
    attitudeSOUT.setTime (t);
  }

  it = SensorsIn.find("joints");
  if (it!=SensorsIn.end())
  {
    const vector<double>& anglesIn = it->second.getValues();
    mlRobotState.resize (anglesIn.size () + 6);
    for (unsigned i = 0; i < 6; ++i)
      mlRobotState (i) = 0.;
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
  ml::Vector zmpGlobal (4);
  for (unsigned int i = 0; i < 3; ++i)
    zmpGlobal(i) = zmpSIN(time + 1)(i);
  zmpGlobal(3) = 1.;
  dgsot::MatrixHomogeneous inversePose;
  freeFlyerPose().inverse(inversePose);
  ml::Vector localZmp = inversePose * zmpGlobal;
  vector<double> ZMPRef(3);
  for(unsigned int i=0;i<3;++i)
    ZMPRef[i] = localZmp(i);

  controlOut["zmp"].setName("zmp");
  controlOut["zmp"].setValues(ZMPRef);

  // Update position of freeflyer in global frame
  for (int i = 0;i < 3; ++i)
    baseff_[i*4+3] = freeFlyerPose () (i, 3);
  for(unsigned i = 0;i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      baseff_[i * 4 + j] = freeFlyerPose () (i, j);

  controlOut["baseff"].setValues(baseff_);
  sotDEBUGOUT(25) ;
}
