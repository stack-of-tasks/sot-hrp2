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

from __future__ import print_function

import numpy as np

#Don't change this order
from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph.ros import RosRobotModel
import pinocchio as se3
from rospkg import RosPack


# Internal helper tool.
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

class Hrp2(AbstractHumanoidRobot):
    """
    This class instanciates a Hrp2 robot
    """

    forceSensorInLeftAnkle =  ((1.,0.,0.,0.),
                               (0.,1.,0.,0.),
                               (0.,0.,1.,-0.105),
                               (0.,0.,0.,1.))
    forceSensorInRightAnkle = ((1.,0.,0.,0.),
                               (0.,1.,0.,0.),
                               (0.,0.,1.,-0.105),
                               (0.,0.,0.,1.))

    accelerometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))

    gyrometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))

    def smallToFull(self, config):
        res = (config + 10*(0.,))
        return res

    def __init__(self, name, robotnumber,
                 device = None, tracer = None):
        
        AbstractHumanoidRobot.__init__ (self, name, tracer)

        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('chest')
        self.device = device
        
        self.AdditionalFrames.append(
            ("accelerometer",
             matrixToTuple(self.accelerometerPosition), "chest"))
        self.AdditionalFrames.append(
            ("gyrometer",
             matrixToTuple(self.gyrometerPosition), "chest"))
        self.AdditionalFrames.append(
            ("leftFootForceSensor",
             self.forceSensorInLeftAnkle, "left-ankle"))
        self.AdditionalFrames.append(
            ("rightFootForceSensor",
             self.forceSensorInRightAnkle, "right-ankle"))
        self.OperationalPointsMap = {'left-wrist'  : 'LARM_JOINT5',
                                     'right-wrist' : 'RARM_JOINT5',
                                     'left-ankle'  : 'LLEG_JOINT5',
                                     'right-ankle' : 'RLEG_JOINT5',
                                     'gaze'        : 'HEAD_JOINT1',
                                     'waist'       : 'WAIST',
                                     'chest'       : 'CHEST_JOINT1'}

        self.dynamic = RosRobotModel("{0}_dynamic".format(name))

        

        rospack = RosPack()
        self.urdfPath = rospack.get_path('hrp2_{0}_description'.format(robotnumber)) + '/urdf/hrp2_{0}.urdf'.format(robotnumber)

        self.pinocchioModel = se3.buildModelFromUrdf(self.urdfPath, se3.JointModelFreeFlyer())
        self.pinocchioData = self.pinocchioModel.createData()
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)
        self.dimension = self.dynamic.getDimension()
        self.plugVelocityFromDevice = True
        if self.dimension != len(self.halfSitting):
            raise RuntimeError("Dimension of half-sitting: {0} differs from dimension of robot: {1}".format (len(self.halfSitting), self.dimension))
        self.initializeRobot()
        self.dynamic.displayModel()
__all__ = [Hrp2]
