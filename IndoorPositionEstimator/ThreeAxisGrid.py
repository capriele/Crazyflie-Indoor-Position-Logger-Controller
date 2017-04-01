#
# Author: Alberto Petrucci (petrucci.alberto@gmail.com) 2017
#
#__author__ = "Alberto Petrucci"
#__copyright__ = "Copyright 2017, Alberto Petrucci"
#__credits__ = ["Alberto Petrucci"]
#__license__ = "Apache"
#__version__ = "1.0.0"
#__maintainer__ = "Alberto Petrucci"
#__email__ = "petrucci.alberto@gmail.com"
#__status__ = "Production"

from __future__ import division
#import direct.directbase.DirectStart
from pandac.PandaModules import *
import sys, os


# Three Axis Coordinate Plane Grid Class (ThreeAxisGrid)
# Mathew Lloyd AKA 'Forklift', August 2008
# 'matthewadamlloyd@gmail.com'
#
#   The grid can be created with any number of axis planes in mind.
#   Simply set size values for the planes you wish to use. Sizes of
#   zero will be ignored. By default, you can create single and three
#   plane setups. Use plane visibility to disable any planes you don't
#   need to acheive a 2 plane setup.
#
#   To create a grid, first create an instance of this class. Then call
#   its 'create' method to create the grid based on the class member
#   variables. 'create' will return a NodePath instance that must be
#   parented to 'render' in order to acheive visibility. Once the grid
#   is created, its settings cannot be changed as the 'create' method
#   generates the geometry procedurally using Panda's LineSeg class.
#    If another grid or a different grid is needed, create a new
#   instance of the ThreeAxisGrid class and setup as described above.
#
#   A 'refresh' method is planned for a future version. This method
#   would allow you to change a ThreeAxisGrid instance's settings,
#   then recreate the geometry without changing the
#   parentNodePath of the instance.
#
# ThreeAxisGrid class member variables are as follows:
#   'xsize' is the x axis length in units of the grid
#   'ysize' is the y axis length in units of the grid
#   'zsize' is the z axis lenght in units of the grid
#   'gridstep' is the spacing in units at which primary grid lines
#      will be drawn
#   'subdiv' is the number used to subdivide the main (gridstep based)
#      grid lines for drawing secondary grid lines example: if the
#      primary grid lines are drawn every 10 units, and subdivision
#      is set at 4, then secondary grid lines will be drawn
#      every 2.5 units
#   'XYPlaneShow' and so forth: used to disable a plane with the
#      creation of 2 plane grids in mind. 1 is on, 0 is off.
#   'endCapLinesShow' is used to turn grid border edges on or off.
#      1 is on, 0 is off.
#   'xaxiscolor' and so forth: axis colors are defaulted to the
#      Maya standard axis colors
#   'gridcolor' is the RGB color of the primary grid lines,
#      defaulted to black
#   'subdivcolor' is the RGB color of the secondary grid lines,
#      defaulted to dark gray
#   'axisThickness' and so forth: sets the thickness of the
#      respective component's lines
#   'parentNode' and 'parentNodePath' are used to contain
#      the three LineSeg instance nodes and paths

class ThreeAxisGrid:
    def __init__(self, xsize=50, ysize=50, zsize=50, gridstep=10, subdiv=10):

        # Init passed variables
        self.XSize = xsize
        self.YSize = ysize
        self.ZSize = zsize
        self.gridStep = gridstep
        self.subdiv = subdiv

        # Init default variables

        # Plane and end cap line visibility (1 is show, 0 is hide)
        self.XYPlaneShow = 1
        self.XZPlaneShow = 1
        self.YZPlaneShow = 1
        self.endCapLinesShow = 1

        # Alpha variables for each plane
        #self.XYPlaneAlpha = 0.1
        #self.XZPlaneAlpha = 0.1
        #self.YZPlaneAlpha = 0.1

        # Colors (RGBA passed as a VBase4 object)
        self.XAxisColor = VBase4(1, 0, 0, 1)
        self.YAxisColor = VBase4(0, 1, 0, 1)
        self.ZAxisColor = VBase4(0, 0, 1, 1)
        self.gridColor = (1, 1, 1, 0.3)
        self.subdivColor = VBase4(.35, .35, .35, 0.1)

        # Line thicknesses (in pixels)
        self.axisThickness = 1
        self.gridThickness = 1
        self.subdivThickness = 1

        # Axis, grid, and subdiv lines must be seperate LineSeg
        # objects in order to allow different thicknesses.
        # The parentNode groups them together for convenience.
        # All may be accessed individually if necessary.
        self.parentNode = None
        self.parentNodePath = None
        self.axisLinesNode = None
        self.axisLinesNodePath = None
        self.gridLinesNode = None
        self.gridLinesNodePath = None
        self.subdivLinesNode = None
        self.subdivLinesNodePath = None

        # Create line objects
        self.axisLines = LineSegs()
        self.subdivLines = LineSegs()

        self.gridLinesXYTop = LineSegs()
        self.gridLinesXYBottom = LineSegs()

        self.gridLinesXZFront = LineSegs()
        self.gridLinesXZBack = LineSegs()

        self.gridLinesYZLeft = LineSegs()
        self.gridLinesYZRight = LineSegs()


        self.gridLinesNodePathXYTop = None
        self.gridLinesNodePathXYBottom = None
        self.gridLinesNodePathXZFront = None
        self.gridLinesNodePathXZBack = None
        self.gridLinesNodePathYZLeft = None
        self.gridLinesNodePathYZRight = None

        self.gridLines = [self.gridLinesXYTop, self.gridLinesXYBottom,
                          self.gridLinesXZFront, self.gridLinesXZBack,
                          self.gridLinesYZLeft, self.gridLinesYZRight]


    def create(self):

        # Set line thicknesses
        self.axisLines.setThickness(self.axisThickness)
        self.subdivLines.setThickness(self.subdivThickness)

        self.gridLinesXYTop.setThickness(self.gridThickness)
        self.gridLinesXYBottom.setThickness(self.gridThickness)
        self.gridLinesXZFront.setThickness(self.gridThickness)
        self.gridLinesXZBack.setThickness(self.gridThickness)
        self.gridLinesYZLeft.setThickness(self.gridThickness)
        self.gridLinesYZRight.setThickness(self.gridThickness)

        if (self.XSize != 0):
            # Draw X axis line
            self.axisLines.setColor(self.XAxisColor)
            self.axisLines.moveTo(-(self.XSize), -self.YSize, -self.ZSize)
            self.axisLines.drawTo(self.XSize, -self.YSize, -self.ZSize)

        if (self.YSize != 0):
            # Draw Y axis line
            self.axisLines.setColor(self.YAxisColor)
            self.axisLines.moveTo(-self.XSize, -self.YSize, -self.ZSize)
            self.axisLines.drawTo(-self.XSize, self.YSize, -self.ZSize)

        if (self.ZSize != 0):
            # Draw Z axis line
            self.axisLines.setColor(self.ZAxisColor)
            self.axisLines.moveTo(-self.XSize, -self.YSize, -self.ZSize)
            self.axisLines.drawTo(-self.XSize, -self.YSize, self.ZSize)

        # Check to see if primary grid lines should be drawn at all
        if (self.gridStep != 0):

            # Draw primary grid lines
            self.gridLinesXYTop.setColor(self.gridColor)
            self.gridLinesXYBottom.setColor(self.gridColor)
            self.gridLinesXZFront.setColor(self.gridColor)
            self.gridLinesXZBack.setColor(self.gridColor)
            self.gridLinesYZLeft.setColor(self.gridColor)
            self.gridLinesYZRight.setColor(self.gridColor)

            # Draw primary grid lines metering x axis if any x-length
            if (self.XSize != 0):

                if ((self.YSize != 0) and (self.XYPlaneShow != 0)):
                    # Draw y lines across x axis starting from center moving out
                    # XY Plane
                    for x in self.myfrange(0, self.XSize, self.gridStep):
                        self.gridLinesXYBottom.moveTo(x, -self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.drawTo(x, self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.moveTo(-x, -self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.drawTo(-x, self.YSize, -self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXYBottom.moveTo(self.XSize, -self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.drawTo(self.XSize, self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.moveTo(-self.XSize, -self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.drawTo(-self.XSize, self.YSize, -self.ZSize)

                    # Draw y lines across x axis starting from center moving out
                    # XY Plane (TOP)
                    for x in self.myfrange(0, self.XSize, self.gridStep):
                        self.gridLinesXYTop.moveTo(x, -self.YSize, self.ZSize)
                        self.gridLinesXYTop.drawTo(x, self.YSize, self.ZSize)
                        self.gridLinesXYTop.moveTo(-x, -self.YSize, self.ZSize)
                        self.gridLinesXYTop.drawTo(-x, self.YSize, self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXYTop.moveTo(self.XSize, -self.YSize, self.ZSize)
                        self.gridLinesXYTop.drawTo(self.XSize, self.YSize, self.ZSize)
                        self.gridLinesXYTop.moveTo(-self.XSize, -self.YSize, self.ZSize)
                        self.gridLinesXYTop.drawTo(-self.XSize, self.YSize, self.ZSize)

                if ((self.ZSize != 0) and (self.XZPlaneShow != 0)):
                    # Draw z lines across x axis starting from center moving out
                    # XZ Plane
                    for x in self.myfrange(0, self.XSize, self.gridStep):
                        self.gridLinesXZBack.moveTo(x, self.YSize, -(self.ZSize))
                        self.gridLinesXZBack.drawTo(x, self.YSize, self.ZSize)
                        self.gridLinesXZBack.moveTo(-x, self.YSize, -(self.ZSize))
                        self.gridLinesXZBack.drawTo(-x, self.YSize, self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXZBack.moveTo(self.XSize, self.YSize, -self.ZSize)
                        self.gridLinesXZBack.drawTo(self.XSize, self.YSize, self.ZSize)
                        self.gridLinesXZBack.moveTo(-self.XSize, self.YSize, -self.ZSize)
                        self.gridLinesXZBack.drawTo(-self.XSize, self.YSize, self.ZSize)

                    # Draw z lines across x axis starting from center moving out
                    # XZ Plane (FRONT)
                    for x in self.myfrange(0, self.XSize, self.gridStep):
                        self.gridLinesXZFront.moveTo(x, -self.YSize, -(self.ZSize))
                        self.gridLinesXZFront.drawTo(x, -self.YSize, self.ZSize)
                        self.gridLinesXZFront.moveTo(-x, -self.YSize, -(self.ZSize))
                        self.gridLinesXZFront.drawTo(-x, -self.YSize, self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXZFront.moveTo(self.XSize, -self.YSize, -self.ZSize)
                        self.gridLinesXZFront.drawTo(self.XSize, -self.YSize, self.ZSize)
                        self.gridLinesXZFront.moveTo(-self.XSize, -self.YSize, -self.ZSize)
                        self.gridLinesXZFront.drawTo(-self.XSize, -self.YSize, self.ZSize)

            # Draw primary grid lines metering y axis if any y-length
            if (self.YSize != 0):

                if ((self.YSize != 0) and (self.XYPlaneShow != 0)):
                    # Draw x lines across y axis
                    # XY Plane
                    for y in self.myfrange(0, self.YSize, self.gridStep):
                        self.gridLinesXYBottom.moveTo(-self.XSize, y, -self.ZSize)
                        self.gridLinesXYBottom.drawTo(self.XSize, y, -self.ZSize)
                        self.gridLinesXYBottom.moveTo(-self.XSize, -y, -self.ZSize)
                        self.gridLinesXYBottom.drawTo(self.XSize, -y, -self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXYBottom.moveTo(-(self.XSize), self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.drawTo(self.XSize, self.YSize, -self.ZSize)
                        self.gridLinesXYBottom.moveTo(-self.XSize, -(self.YSize), -self.ZSize)
                        self.gridLinesXYBottom.drawTo(self.XSize, -(self.YSize), -self.ZSize)

                    # Draw x lines across y axis
                    # XY Plane (TOP)
                    for y in self.myfrange(0, self.YSize, self.gridStep):
                        self.gridLinesXYTop.moveTo(-self.XSize, y, self.ZSize)
                        self.gridLinesXYTop.drawTo(self.XSize, y, self.ZSize)
                        self.gridLinesXYTop.moveTo(-self.XSize, -y, self.ZSize)
                        self.gridLinesXYTop.drawTo(self.XSize, -y, self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXYTop.moveTo(-(self.XSize), self.YSize, self.ZSize)
                        self.gridLinesXYTop.drawTo(self.XSize, self.YSize, self.ZSize)
                        self.gridLinesXYTop.moveTo(-self.XSize, -(self.YSize), self.ZSize)
                        self.gridLinesXYTop.drawTo(self.XSize, -(self.YSize), self.ZSize)

                if ((self.ZSize != 0) and (self.YZPlaneShow != 0)):
                    # Draw z lines across y axis
                    # YZ Plane
                    for y in self.myfrange(0, self.YSize, self.gridStep):
                        self.gridLinesYZLeft.moveTo(-self.XSize, y, -(self.ZSize))
                        self.gridLinesYZLeft.drawTo(-self.XSize, y, self.ZSize)
                        self.gridLinesYZLeft.moveTo(-self.XSize, -y, -self.ZSize)
                        self.gridLinesYZLeft.drawTo(-self.XSize, -y, self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesYZLeft.moveTo(-self.XSize, self.YSize, -(self.ZSize))
                        self.gridLinesYZLeft.drawTo(-self.XSize, self.YSize, self.ZSize)
                        self.gridLinesYZLeft.moveTo(-self.XSize, -(self.YSize), -(self.ZSize))
                        self.gridLinesYZLeft.drawTo(-self.XSize, -(self.YSize), self.ZSize)

                    # Draw z lines across y axis
                    # YZ Plane (RIGHT)
                    for y in self.myfrange(0, self.YSize, self.gridStep):
                        self.gridLinesYZRight.moveTo(self.XSize, y, -self.ZSize)
                        self.gridLinesYZRight.drawTo(self.XSize, y, self.ZSize)
                        self.gridLinesYZRight.moveTo(self.XSize, -y, -self.ZSize)
                        self.gridLinesYZRight.drawTo(self.XSize, -y, self.ZSize)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesYZRight.moveTo(self.XSize, self.YSize, -self.ZSize)
                        self.gridLinesYZRight.drawTo(self.XSize, self.YSize, self.ZSize)
                        self.gridLinesYZRight.moveTo(self.XSize, -(self.YSize), -(self.ZSize))
                        self.gridLinesYZRight.drawTo(self.XSize, -(self.YSize), self.ZSize)

            # Draw primary grid lines metering z axis if any z-length
            if (self.ZSize != 0):

                if ((self.XSize != 0) and (self.XZPlaneShow != 0)):
                    # Draw x lines across z axis
                    # XZ Plane
                    for z in self.myfrange(0, self.ZSize, self.gridStep):
                        self.gridLinesXZBack.moveTo(-(self.XSize), self.YSize, z)
                        self.gridLinesXZBack.drawTo(self.XSize, self.YSize, z)
                        self.gridLinesXZBack.moveTo(-(self.XSize), self.YSize, -z)
                        self.gridLinesXZBack.drawTo(self.XSize, self.YSize, -z)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXZBack.moveTo(-(self.XSize), self.YSize, self.ZSize)
                        self.gridLinesXZBack.drawTo(self.XSize, self.YSize, self.ZSize)
                        self.gridLinesXZBack.moveTo(-(self.XSize), self.YSize, -(self.ZSize))
                        self.gridLinesXZBack.drawTo(self.XSize, self.YSize, -(self.ZSize))

                    # Draw x lines across z axis
                    # XZ Plane (FRONT)
                    for z in self.myfrange(0, self.ZSize, self.gridStep):
                        self.gridLinesXZFront.moveTo(-(self.XSize), -self.YSize, z)
                        self.gridLinesXZFront.drawTo(self.XSize, -self.YSize, z)
                        self.gridLinesXZFront.moveTo(-(self.XSize), -self.YSize, -z)
                        self.gridLinesXZFront.drawTo(self.XSize, -self.YSize, -z)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesXZFront.moveTo(-(self.XSize), -self.YSize, self.ZSize)
                        self.gridLinesXZFront.drawTo(self.XSize, -self.YSize, self.ZSize)
                        self.gridLinesXZFront.moveTo(-(self.XSize), -self.YSize, -(self.ZSize))
                        self.gridLinesXZFront.drawTo(self.XSize, -self.YSize, -(self.ZSize))

                if ((self.YSize != 0) and (self.YZPlaneShow != 0)):
                    # Draw y lines across z axis
                    # YZ Plane
                    for z in self.myfrange(0, self.ZSize, self.gridStep):
                        self.gridLinesYZLeft.moveTo(-self.XSize, -(self.YSize), z)
                        self.gridLinesYZLeft.drawTo(-self.XSize, self.YSize, z)
                        self.gridLinesYZLeft.moveTo(-self.XSize, -(self.YSize), -z)
                        self.gridLinesYZLeft.drawTo(-self.XSize, self.YSize, -z)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesYZLeft.moveTo(-self.XSize, -(self.YSize), self.ZSize)
                        self.gridLinesYZLeft.drawTo(-self.XSize, self.YSize, self.ZSize)
                        self.gridLinesYZLeft.moveTo(-self.XSize, -(self.YSize), -(self.ZSize))
                        self.gridLinesYZLeft.drawTo(-self.XSize, self.YSize, -(self.ZSize))

                    # Draw y lines across z axis
                    # YZ Plane (RIGHT)
                    for z in self.myfrange(0, self.ZSize, self.gridStep):
                        self.gridLinesYZRight.moveTo(self.XSize, -self.YSize, z)
                        self.gridLinesYZRight.drawTo(self.XSize, self.YSize, z)
                        self.gridLinesYZRight.moveTo(self.XSize, -self.YSize, -z)
                        self.gridLinesYZRight.drawTo(self.XSize, self.YSize, -z)

                    if (self.endCapLinesShow != 0):
                        # Draw endcap lines
                        self.gridLinesYZRight.moveTo(self.XSize, -self.YSize, self.ZSize)
                        self.gridLinesYZRight.drawTo(self.XSize, self.YSize, self.ZSize)
                        self.gridLinesYZRight.moveTo(self.XSize, -self.YSize, -self.ZSize)
                        self.gridLinesYZRight.drawTo(self.XSize, self.YSize, -self.ZSize)

        # Check to see if secondary grid lines should be drawn
        if (self.subdiv != 0):

            # Draw secondary grid lines
            self.subdivLines.setColor(self.subdivColor)

            if (self.XSize != 0):
                adjustedstep = self.gridStep / self.subdiv
                print(self.gridStep)
                print(self.subdiv)
                print(adjustedstep)
                print(self.gridStep / self.subdiv)

                if ((self.YSize != 0) and (self.XYPlaneShow != 0)):
                    # Draw y lines across x axis starting from center moving out
                    # XY
                    for x in self.myfrange(0, self.XSize, adjustedstep):
                        self.subdivLines.moveTo(x, -(self.YSize), -self.ZSize)
                        self.subdivLines.drawTo(x, self.YSize, -self.ZSize)
                        self.subdivLines.moveTo(-x, -(self.YSize), -self.ZSize)
                        self.subdivLines.drawTo(-x, self.YSize, -self.ZSize)

                if ((self.ZSize != 0) and (self.XZPlaneShow != 0)):
                    # Draw z lines across x axis starting from center moving out
                    # XZ
                    for x in self.myfrange(0, self.XSize, adjustedstep):
                        self.subdivLines.moveTo(x, 0, -(self.ZSize))
                        self.subdivLines.drawTo(x, 0, self.ZSize)
                        self.subdivLines.moveTo(-x, 0, -(self.ZSize))
                        self.subdivLines.drawTo(-x, 0, self.ZSize)

            if (self.YSize != 0):

                if ((self.YSize != 0) and (self.XYPlaneShow != 0)):
                    # Draw x lines across y axis
                    # XY
                    for y in self.myfrange(0, self.YSize, adjustedstep):
                        self.subdivLines.moveTo(-(self.XSize), y, -self.ZSize)
                        self.subdivLines.drawTo(self.XSize, y, -self.ZSize)
                        self.subdivLines.moveTo(-(self.XSize), -y, -self.ZSize)
                        self.subdivLines.drawTo(self.XSize, -y, -self.ZSize)

                if ((self.ZSize != 0) and (self.YZPlaneShow != 0)):
                    # Draw z lines across y axis
                    # YZ
                    for y in self.myfrange(0, self.YSize, adjustedstep):
                        self.subdivLines.moveTo(-self.XSize, y, -(self.ZSize))
                        self.subdivLines.drawTo(-self.XSize, y, self.ZSize)
                        self.subdivLines.moveTo(-self.XSize, -y, -(self.ZSize))
                        self.subdivLines.drawTo(-self.XSize, -y, self.ZSize)

            if (self.ZSize != 0):

                if ((self.XSize != 0) and (self.XZPlaneShow != 0)):
                    # Draw x lines across z axis
                    # XZ
                    for z in self.myfrange(0, self.ZSize, adjustedstep):
                        self.subdivLines.moveTo(-self.XSize, self.YSize, z)
                        self.subdivLines.drawTo(self.XSize, self.YSize, z)
                        self.subdivLines.moveTo(-self.XSize, self.YSize, -z)
                        self.subdivLines.drawTo(self.XSize, self.YSize, -z)

                if ((self.YSize != 0) and (self.YZPlaneShow != 0)):
                    # Draw y lines across z axis
                    # YZ
                    for z in self.myfrange(0, self.ZSize, adjustedstep):
                        self.subdivLines.moveTo(-self.XSize, -(self.YSize), z)
                        self.subdivLines.drawTo(-self.XSize, self.YSize, z)
                        self.subdivLines.moveTo(-self.XSize, -(self.YSize), -z)
                        self.subdivLines.drawTo(-self.XSize, self.YSize, -z)

        # Create ThreeAxisGrid nodes and nodepaths
        # Create parent node and path
        self.parentNode = PandaNode('threeaxisgrid-parentnode')
        self.parentNodePath = NodePath(self.parentNode)

        # Create axis lines node and path, then reparent
        self.axisLinesNode = self.axisLines.create()
        self.axisLinesNodePath = NodePath(self.axisLinesNode)
        self.axisLinesNodePath.reparentTo(self.parentNodePath)

        # Create grid lines node and path, then reparent
        #self.gridLinesNode = self.gridLines.create()
        #self.gridLinesNodePath = NodePath(self.gridLinesNode)
        #self.gridLinesNodePath.reparentTo(self.parentNodePath)

        gridLinesNode = self.gridLinesXYTop.create()
        self.gridLinesNodePathXYTop = NodePath(gridLinesNode)
        self.gridLinesNodePathXYTop.reparentTo(self.parentNodePath)

        gridLinesNode = self.gridLinesXYBottom.create()
        self.gridLinesNodePathXYBottom = NodePath(gridLinesNode)
        self.gridLinesNodePathXYBottom.reparentTo(self.parentNodePath)

        gridLinesNode = self.gridLinesXZFront.create()
        self.gridLinesNodePathXZFront = NodePath(gridLinesNode)
        self.gridLinesNodePathXZFront.reparentTo(self.parentNodePath)

        gridLinesNode = self.gridLinesXZBack.create()
        self.gridLinesNodePathXZBack = NodePath(gridLinesNode)
        self.gridLinesNodePathXZBack.reparentTo(self.parentNodePath)

        gridLinesNode = self.gridLinesYZLeft.create()
        self.gridLinesNodePathYZLeft = NodePath(gridLinesNode)
        self.gridLinesNodePathYZLeft.reparentTo(self.parentNodePath)

        gridLinesNode = self.gridLinesYZRight.create()
        self.gridLinesNodePathYZRight = NodePath(gridLinesNode)
        self.gridLinesNodePathYZRight.reparentTo(self.parentNodePath)


        # Create subdivision lines node and path then reparent
        self.subdivLinesNode = self.subdivLines.create()
        self.subdivLinesNodePath = NodePath(self.subdivLinesNode)
        self.subdivLinesNodePath.reparentTo(self.parentNodePath)

        return self.parentNodePath

    # Thanks to Edvard Majakari for this float-accepting range method
    def myfrange(self, start, stop=None, step=None):
        if stop is None:
            stop = float(start)
            start = 0.0
        if step is None:
            step = 1.0
        cur = float(start)
        while cur < stop:
            yield cur
            cur += step

    def hideFrontFace(self, angleY=0, angleZ=0):
        self.gridLinesNodePathXZFront.show()
        self.gridLinesNodePathYZRight.show()
        self.gridLinesNodePathXZBack.show()
        self.gridLinesNodePathYZLeft.show()
        if angleY == 0 and angleZ == 0:
            #Front
            self.gridLinesNodePathXZFront.hide()
            return 1
        if angleY == 0 and angleZ == 45:
            #Front + Right
            self.gridLinesNodePathXZFront.hide()
            self.gridLinesNodePathYZRight.hide()
        elif angleY == 0 and angleZ == 90:
            #Right
            self.gridLinesNodePathYZRight.hide()
            return 5
        elif angleY == 0 and angleZ == 135:
            #Back + Right
            self.gridLinesNodePathXZBack.hide()
            self.gridLinesNodePathYZRight.hide()
        elif angleY == 0 and angleZ == 180:
            #Back
            self.gridLinesNodePathXZBack.hide()
            return 3
        elif angleY == 0 and angleZ == 225:
            #Left + Back
            self.gridLinesNodePathYZLeft.hide()
            self.gridLinesNodePathXZBack.hide()
        elif angleY == 0 and angleZ == 270:
            #Left
            self.gridLinesNodePathYZLeft.hide()
            return 6
        elif angleY == 0 and angleZ == 315:
            #Left + Front
            self.gridLinesNodePathXZFront.hide()
            self.gridLinesNodePathYZLeft.hide()
        return 0



#grid = ThreeAxisGrid()
#gridnodepath = grid.create()
#gridnodepath.reparentTo(render)

#run()