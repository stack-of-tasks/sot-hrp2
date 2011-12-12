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
#include <dynamic-graph/corba/interpreter.hh>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>

namespace dgsot=dynamicgraph::sot;

class SoTHRP2Controller: public 
  dgsot::AbstractSotExternalInterface,
  dgsot::Device
{
 public:

  static const std::string LOG_PYTHON;
  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;
  
  virtual const std::string& getClassName () const		
  {  
    return CLASS_NAME;							    
  }

  SoTHRP2Controller(std::string RobotName);
  virtual ~SoTHRP2Controller();

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);
  
  void runPython(std::ostream& file,
		 const std::string& command,
		 dynamicgraph::corba::Interpreter& interpreter);
  
  virtual void startupPython();
  
  
  /// \brief Current integration step.
  double timestep_;
  
  /// \brief Previous robot configuration.
  maal::boost::Vector previousState_;
  
  /// \brief Robot state provided by OpenHRP.
  ///
  /// This corresponds to the real encoders values and take into
  /// account the stabilization step. Therefore, this usually
  /// does *not* match the state control input signal.
  ///
  dynamicgraph::Signal<ml::Vector, int> robotState_;
  
  /// Embedded python interpreter accessible via Corba
  dynamicgraph::corba::Interpreter interpreter_;


};

#endif /* _SOT_HRP2Controller_H_ */
