# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
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
print("Prologue HRP-2")

from dynamic_graph import plug
from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.dynamics.solver import Solver

def prologue_hrp2(robot,device):

    # Initialize the zmp signal to the current com.
    _com = robot.com.value
 
    # Create a solver.
    solver = Solver(robot)

    # Make sure com and feet desired positions match the current
    # positions.
    s = ['left-ankle', 'right-ankle']
    for i in s:
        robot.dynamic.signal(i).recompute(robot.dynamic.signal(i).time + 1)
        robot.features[i].reference.value = \
        robot.dynamic.signal(i).value
        robot.features[i]._feature.selec.value = '111111'
        robot.tasks[i].controlGain.value = 1.

    robot.comRef.value = robot.dynamic.com.value
    robot.device.zmp.value = (0., 0., 0.,)

    # Push com and feet tasks.
    #
    # The robot is currently in half-sitting, so this script freezes com
    # and feet position so that the robot will remain stable while the
    # user program is starting.
    solver.push(robot.tasks ['com'])
    for i in s:
        solver.push(robot.tasks[i])

    print("Prologue ran successfully.")

    # Make sure only robot and solver are visible from the outside.
    return solver


####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
