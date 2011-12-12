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

#define ROBOTNAME std::string("HRP2LAAS")

#include "sot-hrp2-14-controller.hh"

const std::string SoTHRP2_14_Controller::LOG_PYTHON_14="/tmp/HRP2Controller_14_python.out";

SoTHRP2_14_Controller::SoTHRP2_14_Controller():
  SoTHRP2Controller(ROBOTNAME)
{
  startupPython();
}

void SoTHRP2_14_Controller::startupPython()
{
  SoTHRP2Controller::startupPython();
  std::ofstream aof(LOG_PYTHON_14.c_str());
  runPython
    (aof,
     "from dynamic_graph.sot.hrp2_14.prologue import robot, solver",
     interpreter_);
  interpreter_.startCorbaServer ("openhrp", "", "stackOfTasks", "");
}

extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTHRP2_14_Controller;
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
