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

#ifndef _SOT_HRP2Controller_H_
#define _SOT_HRP2Controller_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic_graph_bridge/ros_interpreter.hh>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>

#include "sot-hrp2-device.hh"

namespace dgsot=dynamicgraph::sot;

class SoTHRP2Controller: public 
  dgsot::AbstractSotExternalInterface
{
 public:

  static const std::string LOG_PYTHON;
  
  SoTHRP2Controller(std::string RobotName);
  virtual ~SoTHRP2Controller();

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  void setSecondOrderIntegration(void);
  void setNoIntegration(void);

  /// Embedded python interpreter accessible via Corba
  boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);
  
  void runPython(std::ostream& file,
		 const std::string& command,
		 dynamicgraph::Interpreter& interpreter);
  
  virtual void startupPython();
    

  SoTHRP2Device device_;
};

#endif /* _SOT_HRP2Controller_H_ */
