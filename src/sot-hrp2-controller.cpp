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



const std::string SoTHRP2Controller::LOG_PYTHON="/tmp/HRP2Controller_python.out";

using namespace std;

SoTHRP2Controller::SoTHRP2Controller(std::string RobotName):
  interpreter_(),
  device_(RobotName)
{

  std::cout << __FILE__ << ":" << __FUNCTION__ <<"(#" 
	    << __LINE__ << " )" << std::endl;

}

SoTHRP2Controller::~SoTHRP2Controller()
{
}

void SoTHRP2Controller::
setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_.setupSetSensors(SensorsIn);
}


void SoTHRP2Controller::
nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_.nominalSetSensors(SensorsIn);
}

void SoTHRP2Controller::
cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  device_.cleanupSetSensors(SensorsIn);
}


void SoTHRP2Controller::
getControl(map<string,dgsot::ControlValues> &controlOut)
{
  device_.getControl(controlOut);
}

void SoTHRP2Controller::
runPython(std::ostream& file,
	  const std::string& command,
	  dynamicgraph::corba::Interpreter& interpreter)
{
  file << ">>> " << command << std::endl;
  std::string value = interpreter.python (command);
  if (value != "None")
    file << value;
}

void SoTHRP2Controller::
startupPython()
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
  aof.close();
}


