# -*- coding: utf-8 -*-
# Copyright 2012, CNRS-LAAS, Florent Lamiraux
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

import numpy as np
from math import sqrt
from dynamic_graph.sot.hrp2_14.robot import Robot
from dynamic_graph.sot.core import RPYToMatrix
from dynamic_graph.sot.tools.se3 import SE3, SO3, R3

robot = Robot ('seqplay')
rpy2matrix = RPYToMatrix ('rpy2matrix')
m = 56.868
g = 9.81

pos = None
zmp = None
hip = None
def convert (filename) :
    """
    Convert a seqplay file in OpenHRP format to sot-tool format
    """
    global pos, zmp, hip
    openhrpPos = np.genfromtxt (filename + '.pos')
    openhrpZmp = np.genfromtxt (filename + '.zmp')
    nbConfig = len (openhrpPos)
    if len (openhrpZmp) != nbConfig :
        raise RuntimeError (filename + ".pos and " + filename +
                            ".zmp have different lengths.")
    try:
        openhrpHip = np.genfromtxt (filename + '.hip')
    except IOError:
        hip = []
        for i in range (len (openhrpPos)):
            hip.append ((openhrpPos [i][0], 0, 0, 0,))
        openhrpHip = np.array (hip)

    if len (openhrpHip) != nbConfig :
        raise RuntimeError (filename + ".pos and " + filename +
                            ".hip have different lengths.")

    t = 1
    featurePos = []
    featureLa = []
    featureRa = []
    featureCom = []
    forceRightFoot = []
    forceLeftFoot = []

    fixedFoot = None
    fixedLeftFoot = None
    fixedRightFoot = None
    for (pos, zmp, hip) in zip (openhrpPos, openhrpZmp, openhrpHip) :
        translation = 3*(0.,)
        config = list (translation + tuple (hip [1:]) + tuple (pos [1:31]))
        robot.dynamic.position.value = tuple (config)
        robot.dynamic.position.time = t
        robot.com.recompute (t)
        robot.leftAnkle.position.recompute (t)
        robot.rightAnkle.position.recompute (t)
        lf = SE3 (robot.leftAnkle.position.value) * R3 (0., 0., -0.105)
        rf = SE3 (robot.rightAnkle.position.value) * R3 (0., 0., -0.105)
        # find support foot
        rpy2matrix.sin.value = tuple (hip [1:])
        rpy2matrix.sout.recompute (t)
        waist = SE3 (rpy2matrix.sout.value, translation)
        zmpGlob = waist * R3 (tuple (zmp [1:]))
        # fr = m g * (zmpGlob - lf | rf - lf)/||rf - lf||^2
        # fl = (m g - fr)
        fr = m * g * ((zmpGlob - lf)*(rf - lf)/((rf - lf)*(rf - lf)))
        fl = m * g - fr
        if (lf - zmpGlob) * (lf - zmpGlob) < (rf - zmpGlob) * (rf - zmpGlob) :
            supportFoot = lf
            fixedFoot = fixedLeftFoot
        else :
            supportFoot = rf
            fixedFoot = fixedRightFoot
        t+=1
        # move support foot to previous value
        if fixedFoot is None:
            config [2] -= supportFoot [2]
        else:
            config [0] += fixedFoot [0] - supportFoot [0]
            config [1] += fixedFoot [1] - supportFoot [1]
            config [2] += fixedFoot [2] - supportFoot [2]

        robot.dynamic.position.value = tuple (config)
        robot.dynamic.position.time = t
        robot.com.recompute (t)
        robot.leftAnkle.position.recompute (t)
        robot.rightAnkle.position.recompute (t)
        featurePos.append (config)
        featureCom.append (robot.com.value)
        featureLa.append (robot.leftAnkle.position.value)
        featureRa.append (robot.rightAnkle.position.value)
        forceLeftFoot.append ((0.,0.,fl,0.,0.,0.,))
        forceRightFoot.append ((0.,0.,fr,0.,0.,0.,))
        t += 1
        fixedLeftFoot = \
            SE3 (robot.leftAnkle.position.value) * R3 (0., 0., -0.105)
        fixedRightFoot = \
            SE3 (robot.rightAnkle.position.value) * R3 (0., 0., -0.105)

    filePos = open (filename + '.posture', 'w')
    fileLa = open (filename + '.la', 'w')
    fileRa = open (filename + '.ra', 'w')
    fileCom = open (filename + '.com', 'w')
    fileFl = open (filename + '.fl', 'w')
    fileFr = open (filename + '.fr', 'w')

    dt = .005
    for (pos, la, ra, com,
         force_lf, force_rf, i) in zip (featurePos, featureLa, featureRa,
                                        featureCom, forceLeftFoot,
                                        forceRightFoot, xrange (10000000)) :
        t = i*dt
        filePos.write ("{0}".format (t))
        fileLa.write ("{0}".format (t))
        fileRa.write ("{0}".format (t))
        fileCom.write ("{0}".format (t))
        fileFl.write ("{0}".format (t))
        fileFr.write ("{0}".format (t))

        for x in pos:
            filePos.write ("\t{0}".format (x))

        for row in la:
            for x in row:
                fileLa.write ("\t{0}".format (x))

        for row in ra:
            for x in row:
                fileRa.write ("\t{0}".format (x))

        for x in com:
            fileCom.write ("\t{0}".format (x))

        for x in force_lf:
            fileFl.write ("\t{0}".format (x))

        for x in force_rf:
            fileFr.write ("\t{0}".format (x))

        filePos.write ("\n")
        fileLa.write ("\n")
        fileRa.write ("\n")
        fileCom.write ("\n")
        fileFl.write ("\n")
        fileFr.write ("\n")

    filePos.close ()
    fileLa.close ()
    fileRa.close ()
    fileCom.close ()
    fileFl.close ()
    fileFr.close ()

if __name__ == '__main__':
    convert ('/opt/grx3.0/HRP2LAAS/etc/walkfwd')
