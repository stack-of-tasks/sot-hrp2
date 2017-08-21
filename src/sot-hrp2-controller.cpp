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
#include <sot/core/exception-abstract.hh>
#include <dynamic_graph_bridge/ros_init.hh>
#include "sot-hrp2-controller.hh"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
const std::string SoTHRP2Controller::LOG_PYTHON="/tmp/HRP2Controller_python.out";

using namespace std;

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;

void workThread(SoTHRP2Controller *aSoTHRP2C)
{
  
  dynamicgraph::Interpreter aLocalInterpreter(dynamicgraph::rosInit(false,true));

  aSoTHRP2C->interpreter_ = 
    boost::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);
  std::cout << "Going through the thread." << std::endl;
  {
    boost::lock_guard<boost::mutex> lock(mut);
    data_ready=true;
  }
  cond.notify_all();
  ros::waitForShutdown();
}

SoTHRP2Controller::SoTHRP2Controller(std::string RobotName):
  device_(RobotName)
{

  std::cout << "Going through SoTHRP2Controller for " <<RobotName<< std::endl;
  boost::thread thr(workThread,this);
  sotDEBUG(25) << __FILE__ << ":" 
	       << __FUNCTION__ <<"(#" 
	       << __LINE__ << " )" << std::endl;

  boost::unique_lock<boost::mutex> lock(mut);
  cond.wait(lock);
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
setSecondOrderIntegration(void)
{
  device_.setSecondOrderIntegration();
}

void SoTHRP2Controller::
setNoIntegration(void)
{
  device_.setNoIntegration();
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
  try 
    {
      sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
      device_.getControl(controlOut);
      sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
    }
  catch ( dynamicgraph::sot::ExceptionAbstract & err)
    {

      std::cout << __FILE__ << " " 
		<< __FUNCTION__ << " (" 
		<< __LINE__ << ") " 
		<< err.getStringMessage() 
		<<  endl;
      throw err;
    }
}

void SoTHRP2Controller::
runPython(std::ostream& file,
	  const std::string& command,
	  dynamicgraph::Interpreter& interpreter)
{
  file << ">>> " << command << std::endl;
  std::string lerr(""),lout(""),lres("");
  interpreter.runCommand(command,lres,lout,lerr);
  if (lres != "None")
    {
      if (lres=="<NULL>")
	{
	  file << lout << std::endl;
	  file << "------" << std::endl;
	  file << lerr << std::endl;
	}
      else
	file << lres << std::endl;
    }
}

void SoTHRP2Controller::
startupPython()
{
  std::ofstream aof(LOG_PYTHON.c_str());
  runPython (aof, "import sys, os", *interpreter_);
  runPython (aof, "pythonpath = os.environ['PYTHONPATH']", *interpreter_);
  runPython (aof, "path = []", *interpreter_);
  runPython (aof,
	     "for p in pythonpath.split(':'):\n"
	     "  if p not in sys.path:\n"
	     "    path.append(p)", *interpreter_);
  runPython (aof, "path.extend(sys.path)", *interpreter_);
  runPython (aof, "sys.path = path", *interpreter_);

  // Calling again rosInit here to start the spinner. It will
  // deal with topics and services callbacks in a separate, non
  // real-time thread. See roscpp documentation for more
  // information.
  dynamicgraph::rosInit (true);

  aof.close();
}
