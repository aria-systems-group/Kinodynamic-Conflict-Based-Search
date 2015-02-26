#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Caleb Voss and Wilson Beebe
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Authors: Caleb Voss, Wilson Beebe


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pylab import *
from matplotlib import cm
import numpy

def readFile(fileName):
    file = open(fileName, "r")
    points = map(lambda line: map(float, line.split()), file.readlines())
    file.close()
    return points

def potential(x,y):
    return 1 + numpy.sin(x)*numpy.sin(y)

def addPotential(points):
    x = map(lambda x: x[0], points)
    y = map(lambda x: x[1], points)
    z = map(lambda p: potential(p[0],p[1]), points)
    return x,y,z

def potentialSurface():
    X = numpy.arange(-8, 8, 0.25)
    Y = numpy.arange(-8, 8, 0.25)
    X, Y = numpy.meshgrid(X, Y)
    Z = potential(X,Y)
    return X,Y,Z

fig = plt.figure()
ax = fig.gca(projection='3d', aspect='equal')
X,Y,Z = potentialSurface()
ax.plot_surface(X,Y,Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0)
x,y,z = addPotential(readFile("vfrrt-conservative.path"))
ax.plot(x,y,z,color='b')
x,y,z = addPotential(readFile("trrt-conservative.path"))
ax.plot(x,y,z,color='r')
x,y,z = addPotential(readFile("rrtstar-conservative.path"))
ax.plot(x,y,z,color='g')
plt.show()
