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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/ros.cmake)

SET(PROJECT_NAME sot-hrp2)
SET(PROJECT_DESCRIPTION "dynamic-graph package for HRP-2 robot")
SET(PROJECT_URL "http://github.com/laas/sot-hrp2")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mathtools")
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.9.0")

ADD_REQUIRED_DEPENDENCY("jrl-dynamics >= 1.19.0")
ADD_REQUIRED_DEPENDENCY("hrp2-dynamics >= 1.5.0")
ADD_REQUIRED_DEPENDENCY("hrp2-10-optimized >= 1.0.1")

ADD_REQUIRED_DEPENDENCY("hrp2-10 >= 1.0.1")
ADD_REQUIRED_DEPENDENCY("hrp2-14 >= 1.8")

ADD_REQUIRED_DEPENDENCY("dynamic-graph")
ADD_REQUIRED_DEPENDENCY("sot-core")
ADD_REQUIRED_DEPENDENCY("sot-dynamic")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")

ADD_ROSPACK_DEPENDENCY("dynamic_graph_bridge")
# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS filesystem system thread)
SEARCH_FOR_BOOST()

# Handle rpath necessary to handle ROS multiplace packages
# libraries inclusion
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Add subdirectories.
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
