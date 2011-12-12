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

#ifndef _SOT_HRP2_14_Controller_H_
#define _SOT_HRP2_14_Controller_H_

#include "sot-hrp2-controller.hh"
namespace dgsot=dynamicgraph::sot;

class SoTHRP2_14_Controller: public SoTHRP2Controller
{
 public:
  static const std::string LOG_PYTHON_14;

  SoTHRP2_14_Controller();
  virtual ~SoTHRP2_14_Controller() {};


 protected:

  virtual void startupPython();
  

};

#endif /* _SOT_HRP2Controller_H_ */
