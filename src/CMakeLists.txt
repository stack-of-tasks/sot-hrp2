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
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

FUNCTION(COMPILE_PLUGIN NAME SOURCES ENTITIES)
  ADD_LIBRARY(${NAME} SHARED ${SOURCES})
  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mathtools)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-dynamics)

  INSTALL(TARGETS ${NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR}/plugin)

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/hrp2/${PYTHON_LIBRARY_NAME}"
    ${NAME}
    sot-hrp2-${PYTHON_LIBRARY_NAME}-wrap
    )
ENDFUNCTION()

# Compile plug-ins.
IF (HRP2_DYNAMICS_FOUND)
  MESSAGE(STATUS "Add dynamic-hrp2_14 plugin")
  COMPILE_PLUGIN(dynamic-hrp2_14 dynamic-hrp2_14.cc DynamicHrp2_14)
  PKG_CONFIG_USE_DEPENDENCY(dynamic-hrp2_14 hrp2-dynamics)
ENDIF()

IF (HRP2_10_OPTIMIZED_FOUND)
  MESSAGE(STATUS "Add dynamic-hrp2_10 plugin")
  COMPILE_PLUGIN(dynamic-hrp2_10 dynamic-hrp2_10.cc DynamicHrp2_10)
  PKG_CONFIG_USE_DEPENDENCY(dynamic-hrp2_10 hrp2-10-optimized)
ENDIF()

IF (HRP2_10_OPTIMIZED_FOUND)
  CONFIG_FILES(dynamic_graph/sot/hrp2_10/robot.py)
ENDIF()

IF (HRP2_DYNAMICS_FOUND)
  CONFIG_FILES(dynamic_graph/sot/hrp2_14/robot.py)
ENDIF()

# Install Python files.
SET(PYTHON_MODULE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/hrp2)
SET(PYTHON_MODULE_BUILD_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/hrp2)

SET(PYTHON_MODULE dynamic_graph/sot/hrp2)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "hrp2.py" )
SET(FILES __init__.py robot.py)

# Install dynamic_graph.sot.hrp2_14
IF (HRP2_DYNAMICS_FOUND)
  SET(PYTHON_MODULE dynamic_graph/sot/hrp2_14)
  PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
  PYTHON_INSTALL_BUILD("${PYTHON_MODULE}" "robot.py" "${PYTHON_SITELIB}")
  PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py")
ENDIF()

# Install dynamic_graph.sot.hrp2_10
IF (HRP2_10_OPTIMIZED_FOUND)
  SET(PYTHON_MODULE dynamic_graph/sot/hrp2_10)
  PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
  PYTHON_INSTALL_BUILD("${PYTHON_MODULE}" "robot.py" "${PYTHON_SITELIB}")
  PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py")
ENDIF()

# Add the library to wrap the device of HRP2.
MACRO(build_hrp2_device)

  SET(DEVICE_NAME sot-hrp2-device)
  ADD_LIBRARY(${DEVICE_NAME}
    SHARED
    sot-hrp2-device.cpp
  )

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "sot-core")

  ROSPACK_USE_DEPENDENCY(${DEVICE_NAME} "dynamic_graph_bridge")

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${DEVICE_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${DEVICE_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/hrp2/${PYTHON_LIBRARY_NAME}"
    ${DEVICE_NAME}
    sot-hrp2-${PYTHON_LIBRARY_NAME}-wrap
    )

ENDMACRO()

build_hrp2_device()

# Add the library to wrap the controller of HRP2.
MACRO(build_hrp2_controller robotnumber)

  SET(CONTROLLER_NAME sot-hrp2-${robotnumber}-controller)
  ADD_LIBRARY(${CONTROLLER_NAME}
    SHARED
    sot-hrp2-controller.cpp
    sot-hrp2-${robotnumber}-controller.cpp
  )

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")

  ROSPACK_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic_graph_bridge")

  ADD_DEPENDENCIES(${CONTROLLER_NAME} "sot-hrp2-device")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "sot-hrp2-device")

  
  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

ENDMACRO()

IF (HRP2_10_OPTIMIZED_FOUND)
  build_hrp2_controller("10")
ENDIF()

IF (HRP2_DYNAMICS_FOUND)
  build_hrp2_controller("14")
ENDIF()

