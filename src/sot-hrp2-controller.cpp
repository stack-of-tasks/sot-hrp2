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

#include <sot/core/debug.hh>

#include "sot-hrp2-controller.hh"


#define ENTITYNAME std::string("DLRBiped")

const std::string SoTHRP2Controller::CLASS_NAME = "Device";
const std::string SoTHRP2Controller::LOG_PYTHON="/tmp/HRP2Controller_python.out";
const double SoTHRP2Controller::TIMESTEP_DEFAULT = 0.005;

using namespace std;

SoTHRP2Controller::SoTHRP2Controller():
  dgsot::Device(ENTITYNAME),
  timestep_ (TIMESTEP_DEFAULT),
  previousState_ (),
  robotState_ ("StackOfTasks(" + ENTITYNAME + ")::output(vector)::robotState")

{
  signalRegistration (robotState_);
  std::cout << __FILE__ << ":" << __FUNCTION__ <<"(#" << __LINE__ << " )" << std::endl;
}

SoTHRP2Controller::~SoTHRP2Controller()
{
}

void SoTHRP2Controller::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  startupPython();

  vector<double>  anglesIn = SensorsIn["joints"].getValues();

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


void SoTHRP2Controller::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
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

void SoTHRP2Controller::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
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


void SoTHRP2Controller::getControl(map<string,dgsot::ControlValues> &controlOut)
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
      baseff[i * 3 + j] = freeFlyerPose () (i, j);

  controlOut["baseff"].setValues(baseff);
  
}

void SoTHRP2Controller::updateRobotState(vector<double> &anglesIn)
{
  ml::Vector robotState (anglesIn.size () + 6);
  for (unsigned i = 0; i < 6; ++i)
    robotState (i) = 0.;
  for (unsigned i = 0; i < anglesIn.size(); ++i)
    robotState (i + 6) = anglesIn[i];
  robotState_.setConstant(robotState);
  
}

void SoTHRP2Controller::runPython(std::ostream& file,
				      const std::string& command,
				      dynamicgraph::corba::Interpreter& interpreter)
{
  file << ">>> " << command << std::endl;
  std::string value = interpreter.python (command);
  if (value != "None")
    file << value;
}

void SoTHRP2Controller::startupPython()
{
  std::cout << "Went through startupPython()" << std::endl;
  std::ofstream aof(LOG_PYTHON.c_str());
  runPython (aof, "import sys, os", interpreter_);
  runPython (aof, "pythonpath = os.environ['PYTHONPATH']", interpreter_);
  runPython (aof, "path = []", interpreter_);
  runPython (aof,
	     "for p in pythonpath.split(':'):\n"
	     "  if p not in sys.path:\n"
	     "    path.append(p)", interpreter_);
  runPython (aof, "path.extend(sys.path)", interpreter_);
  runPython (aof, "sys.path = path", interpreter_);
  runPython
    (aof,
     "from dynamic_graph.sot.openhrp.prologue_dlr_biped import robot, solver",
     interpreter_);
  interpreter_.startCorbaServer ("openhrp", "", "stackOfTasks", "");
}

extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTHRP2Controller;
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
