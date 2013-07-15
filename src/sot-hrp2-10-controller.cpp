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

#define ROBOTNAME std::string("HRP2JRL")

#include "sot-hrp2-10-controller.hh"

const std::string SoTHRP2_10_Controller::LOG_PYTHON_10="/tmp/HRP2Controller_10_python.out";

SoTHRP2_10_Controller::SoTHRP2_10_Controller():
  SoTHRP2Controller(ROBOTNAME)
{
  startupPython();
  interpreter_->startRosService ();
}

void SoTHRP2_10_Controller::startupPython()
{
  SoTHRP2Controller::startupPython();

  std::ofstream aof(LOG_PYTHON_10.c_str());
  runPython
    (aof,
     "from dynamic_graph.sot.hrp2_10.prologue import robot, solver",
     *interpreter_);
  aof.close();
}

extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTHRP2_10_Controller;
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
