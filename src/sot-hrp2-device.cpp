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
 * that you either purchased the proper license to havec access to
 * those informations, or that you signed the appropriate
 * Non-Disclosure agreement.
 *
 *
 */
#include <map>

#include <sot/core/debug.hh>

#include "sot-hrp2-device.hh"

using namespace std;

const std::string SoTHRP2Device::CLASS_NAME = "Device";
const double SoTHRP2Device::TIMESTEP_DEFAULT = 0.005;

SoTHRP2Device::SoTHRP2Device(std::string RobotName):
  dgsot::Device(RobotName),
  timestep_(TIMESTEP_DEFAULT),
  previousState_ (),
  robotState_ ("StackOfTasks(" + RobotName + ")::output(vector)::robotState")
{
  signalRegistration (robotState_);
}

SoTHRP2Device::~SoTHRP2Device()
{ }

void SoTHRP2Device::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
 
  vector<double> anglesIn = SensorsIn["joints"].getValues();

  // Read state from motor command
  int t = stateSOUT.getTime () + 1;
  maal::boost::Vector state = stateSOUT.access (t);

  for (unsigned int i = 6; i < state.size (); ++i)
    state (i) = anglesIn[i - 6];

  previousState_ = state;
  
  sotDEBUG (25) << "state = " << state << std::endl;
  stateSOUT.setConstant (state);

  // Implements force recollection.
  ml::Vector mlforces(6);
  vector<double> forcesIn = SensorsIn["forces"].getValues();
  for(int i=0;i<4;++i)
    {
      for(int j=0;j<6;++j)
	mlforces(j) = forcesIn[i*6+j];
      forcesSOUT[i]->setConstant(mlforces);
    }
      
  updateRobotState(anglesIn);
}


void SoTHRP2Device::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  // Read angular values.
  vector<double>  anglesIn = SensorsIn["joints"].getValues();
  updateRobotState(anglesIn);

  // Read force values.
  vector<double> forcesIn = SensorsIn["forces"].getValues();
  ml::Vector mlforces (6);
  int t = controlSIN.getTime();
  for(int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 6; ++j)
	mlforces(j) = forcesIn[i*6+j];
      forcesSOUT[i]->setConstant (mlforces);
      forcesSOUT[i]->setTime(t+1);
    }
  
}

void SoTHRP2Device::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  vector<double>  anglesIn = SensorsIn["joints"].getValues();

  updateRobotState(anglesIn);

  ml::Vector mlforces (6);
  for(int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 6; ++j)
	mlforces(j) = 0.0;
      forcesSOUT[i]->setConstant (mlforces);
    }
}


void SoTHRP2Device::getControl(map<string,dgsot::ControlValues> &controlOut)
{
  vector<double> anglesOut;
  anglesOut.resize(state_.size());

  // Integrate control
  try 
    {
      increment(timestep_);
    }
  catch(dynamicgraph::ExceptionSignal)
    {
      // The strategy in case of problem is to maintain the previous
      // state.
      for(unsigned int i=6; i < state_.size();++i)
	anglesOut[i-6] = previousState_(i);

      controlOut["joints"].setValues(anglesOut);
      std::cout << "Signal from the control. Freezing the state." << std::endl;
      return;
    }

  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << state_ - previousState_ << std::endl;
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
  std::vector<double> baseff;
  baseff.resize(12);
  for (int i = 0;i < 3; ++i)
    baseff[i*4+3] = freeFlyerPose () (i, 3);
  for(unsigned i = 0;i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      baseff[i * 4 + j] = freeFlyerPose () (i, j);

  controlOut["baseff"].setValues(baseff);
  
}

void SoTHRP2Device::updateRobotState(vector<double> &anglesIn)
{
  ml::Vector robotState (anglesIn.size () + 6);
  for (unsigned i = 0; i < 6; ++i)
    robotState (i) = 0.;
  for (unsigned i = 0; i < anglesIn.size(); ++i)
    robotState (i + 6) = anglesIn[i];
  robotState_.setConstant(robotState);
  
}

