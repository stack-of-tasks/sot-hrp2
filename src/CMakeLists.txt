# Copyright 2011, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-hrp2.
# sot-hrp2 is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-hrp2 is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-hrp2. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(../cmake/python.cmake)

FUNCTION(COMPILE_PLUGIN NAME SOURCES ENTITIES)
  ADD_LIBRARY(${NAME} SHARED ${SOURCES})
  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mathtools)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)

  INSTALL(TARGETS ${NAME} DESTINATION lib/plugin)

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
  ADD_DEPENDENCIES(${NAME} MKDIR_${PYTHON_LIBRARY_NAME})
  ADD_CUSTOM_TARGET(MKDIR_${PYTHON_LIBRARY_NAME}
    mkdir -p dynamic_graph/sot/hrp2/${PYTHON_LIBRARY_NAME}
    )
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/hrp2/${PYTHON_LIBRARY_NAME}"
    ${NAME}
    sot/hrp2/${PYTHON_LIBRARY_NAME}/wrap
    )
ENDFUNCTION()

# Compile plug-ins.
COMPILE_PLUGIN(dynamic-hrp2_14 dynamic-hrp2_14.cc DynamicHrp2_14)
COMPILE_PLUGIN(dynamic-hrp2_10 dynamic-hrp2_10.cc DynamicHrp2_10)


CONFIG_FILES(dynamic_graph/sot/hrp2_10/robot.py)
CONFIG_FILES(dynamic_graph/sot/hrp2_14/robot.py)

# Install Python files.
SET(PYTHON_MODULE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/hrp2)
SET(PYTHON_MODULE_BUILD_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/hrp2)

SET(PYTHON_MODULE dynamic_graph/sot/hrp2)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "hrp2.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py" )
SET(FILES __init__.py robot.py)

# Install dynamic_graph.sot.hrp2_14
SET(PYTHON_MODULE dynamic_graph/sot/hrp2_14)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_BUILD("${PYTHON_MODULE}" "robot.py" ${PYTHON_SITELIB})
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py")

# Install dynamic_graph.sot.hrp2_10
SET(PYTHON_MODULE dynamic_graph/sot/hrp2_10)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_BUILD("${PYTHON_MODULE}" "robot.py" ${PYTHON_SITELIB})
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py")

# Add the library to wrap the controller of HRP2-JRL.
MACRO(build_hrp2_controller robotnumber)

ADD_LIBRARY(sot-hrp2-${robotnumber}-controller
  SHARED
  sot-hrp2-controller.cpp
  sot-hrp2-${robotnumber}-controller.cpp
)

# Link the dynamic library containing the SoT with its dependencies.
PKG_CONFIG_USE_DEPENDENCY(sot-hrp2-${robotnumber}-controller "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(sot-hrp2-${robotnumber}-controller "sot-core")
PKG_CONFIG_USE_DEPENDENCY(sot-hrp2-${robotnumber}-controller "dynamic-graph-corba")

INSTALL(TARGETS sot-hrp2-${robotnumber}-controller DESTINATION lib)
ENDMACRO()

build_hrp2_controller("10")
build_hrp2_controller("14")