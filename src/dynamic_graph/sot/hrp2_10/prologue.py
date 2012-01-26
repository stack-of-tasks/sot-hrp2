# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.
print("Prologue HRP2-10")

from dynamic_graph.sot.hrp2_10.robot import *
from dynamic_graph.sot.hrp2.prologue import *

# Create the device.
# This entity behaves exactly like robotsimu except:
# 1. it does not provide the increment method
# 2. it forwards the robot control to the sot-abstract
#    controller.
Device = PyEntityFactoryClass('Device')
# Create instance for HRP2JRL
device=Device('HRP2JRL')

# Create the robot using the device.
robot = Robot(name = 'robot', device = device)

solver = prologue_hrp2(robot,device)
__all__ = ["robot","solver"]

####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
