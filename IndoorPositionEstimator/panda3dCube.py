#!/usr/bin/env python
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

from ThreeAxisGrid import ThreeAxisGrid
from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.DirectGui import *
from pandac.PandaModules import *
from direct.interval.IntervalGlobal import *
from panda3d.core import lookAt
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter
from panda3d.core import Texture, GeomNode
from panda3d.core import PerspectiveLens
from panda3d.core import CardMaker
from panda3d.core import Light, Spotlight
from panda3d.core import TextNode
from panda3d.core import LVector3, Point3
from panda3d.core import TransparencyAttrib
from panda3d.core import ConfigVariableString
import sys
import json

configurations = False
crazyfliesLink = []
with open('config.json') as data_file:
    configurations = json.load(data_file)
    crazyflies = configurations['crazyflies']
    for crazyflie in crazyflies:
        link = crazyflie['link']
        crazyfliesLink.append(link)
with open('sequence.json') as data_file:
    jsonFile = json.load(data_file)
NodesPositions = configurations['nodes']
Sequence = jsonFile

# Menu
from Menu import *

# CrazyFlie libs
import cflib.crtp
from cflib.logger import Logger

from pandac.PandaModules import loadPrcFileData
ConfigVariableString('window-title', 'CrazyFlie Logger + Controller')
loadPrcFileData("", "Object Locator - Alberto Petrucci 2017")
loadPrcFileData("", "fullscreen 0") # Set to 1 for fullscreen
loadPrcFileData("", "win-size 600 600")
loadPrcFileData("", "win-origin 10 10")

base = ShowBase()
base.disableMouse()
cameraPivot = render.attachNewNode("CameraPivot")
base.camera.reparentTo(cameraPivot)
base.camera.setPos(0, -360, 0)
base.setBackgroundColor(0.5, 0.1, 0.5)

# You can't normalize inline so this is a helper function
def normalized(*args):
    myVec = LVector3(*args)
    myVec.normalize()
    return myVec

# helper function to make a square given the Lower-Left-Hand and
# Upper-Right-Hand corners

def makeSquare(x1, y1, z1, x2, y2, z2, c=[0.0, 0.0, 0.0, 1.0]):
    format = GeomVertexFormat.getV3n3cpt2()
    vdata = GeomVertexData('square', format, Geom.UHDynamic)

    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    texcoord = GeomVertexWriter(vdata, 'texcoord')

    # make sure we draw the sqaure in the right plane
    if x1 != x2:
        vertex.addData3(x1, y1, z1)
        vertex.addData3(x2, y1, z1)
        vertex.addData3(x2, y2, z2)
        vertex.addData3(x1, y2, z2)

        normal.addData3(normalized(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y1 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
        normal.addData3(normalized(2 * x1 - 1, 2 * y2 - 1, 2 * z2 - 1))

    else:
        vertex.addData3(x1, y1, z1)
        vertex.addData3(x2, y2, z1)
        vertex.addData3(x2, y2, z2)
        vertex.addData3(x1, y1, z2)

        normal.addData3(normalized(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y2 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
        normal.addData3(normalized(2 * x1 - 1, 2 * y1 - 1, 2 * z2 - 1))

    # adding different colors to the vertex for visibility
    color.addData4f(c[0], c[1], c[2], c[3])
    color.addData4f(c[0], c[1], c[2], c[3])
    color.addData4f(c[0], c[1], c[2], c[3])
    color.addData4f(c[0], c[1], c[2], c[3])

    texcoord.addData2f(0.0, 1.0)
    texcoord.addData2f(0.0, 0.0)
    texcoord.addData2f(1.0, 0.0)
    texcoord.addData2f(1.0, 1.0)

    # Quads aren't directly supported by the Geom interface
    # you might be interested in the CardMaker class if you are
    # interested in rectangle though
    tris = GeomTriangles(Geom.UHDynamic)
    tris.addVertices(0, 1, 3)
    tris.addVertices(1, 2, 3)

    square = Geom(vdata)
    square.addPrimitive(tris)
    return square


class MyTapper(DirectObject):

    def __init__(self):
        self.testTexture = loader.loadTexture("maps/envir-reeds.png")
        self.accept("l", self.toggleLightsSide)
        self.accept("+", self.setKey, ["forward", 1])
        self.accept("-", self.setKey, ["backward", 1])
        self.accept("escape", self.closeAll)
        self.accept("arrow_left", self.setKey, ["left", 1])
        self.accept("arrow_right", self.setKey, ["right", 1])
        self.accept("arrow_up", self.setKey, ["top", 1])
        self.accept("arrow_down", self.setKey, ["down", 1])
        self.accept("enter", self.start_sequence)
        self.accept("q", self.emergency_stop)
        self.accept("l", self.landing)

        self.cameraAngles = [0, -360, 0]

        render.setTransparency(TransparencyAttrib.MAlpha)

        self.grid = ThreeAxisGrid(xsize=63, ysize=63, zsize=63, gridstep=13, subdiv=0)
        gridnodepath = self.grid.create()
        gridnodepath.wrtReparentTo(render)
        # Nascondo la faccia frontale
        self.grid.hideFrontFace(self.cameraAngles[2] % 360, self.cameraAngles[0]%360)

        # Posiziono i Nodi
        self.placeNodes(NodesPositions)

        # Vettore contenente tutti i riferimenti ai cubi
        self.cubes = []

        self.LightsOn = False
        self.LightsOn1 = False
        slight = Spotlight('slight')
        slight.setColor((1, 1, 1, 1))
        lens = PerspectiveLens()
        slight.setLens(lens)
        self.slnp = render.attachNewNode(slight)
        self.slnp1 = render.attachNewNode(slight)

        # Reference system
        position = [-73, -73, -73]
        self.cameraRS = render.attachNewNode("CameraRS")
        self.cameraRSRot = render.attachNewNode("CameraRSRot")
        self.cameraRS.setPos((6.8+position[0]+position[0]-0.1+position[0])/3,
                             (-6.9+position[1]+position[1]-1.1+position[1])/3,
                             (0.1+position[2]+position[2]+6.9+position[2])/3)
        self.cameraRS.setHpr(0, 0, 0)
        self.cameraRS.wrtReparentTo(base.cam)

        self.zCone = loader.loadModel("models/cone.egg")
        self.zCone.setScale(0.02, 0.05, 0.02)
        self.zCone.setPos(-0.1+position[0], -1.1+position[1], 6.9+position[2])
        self.zCone.setHpr(0, 90, 0)
        self.zCone.setColor((0, 0, 1, 1))
        self.zCone.wrtReparentTo(self.cameraRS)

        self.yCone = loader.loadModel("models/cone.egg")
        self.yCone.setScale(0.02, 0.05, 0.02)
        self.yCone.setPos(position[0], position[1], position[2])
        self.yCone.setHpr(0, 0, 0)
        self.yCone.setColor((0, 1, 0, 1))
        self.yCone.wrtReparentTo(self.cameraRS)

        self.xCone = loader.loadModel("models/cone.egg")
        self.xCone.setScale(0.02, 0.05, 0.02)
        self.xCone.setPos(6.8+position[0], -6.9+position[1], 0.1+position[2])
        self.xCone.setHpr(-90, 0, -90)
        self.xCone.setColor((1, 0, 0, 1))
        self.xCone.wrtReparentTo(self.cameraRS)

        # Turn on light
        # self.toggleLightsSide()

        # Menu
        self.gameMenu = DropDownMenu(
           items=(
             ('_Menu', self.createMenuItems),
             ('_Help', self.createHelpMenuItems)
           ),
           sidePad=.75,
           align=DropDownMenu.ALeft,
           # align=DropDownMenu.ACenter,
           # align=DropDownMenu.ARight,
           # effect=DropDownMenu.ESlide,
           effect=DropDownMenu.EStretch,
           # effect=DropDownMenu.EFade,
           edgePos=DropDownMenu.PTop,
           baselineOffset=-.35,
           scale=.045, itemHeight=1.2, leftPad=.2,
           separatorHeight=.3,
           underscoreThickness=1,
           BGColor=(.9, .9, .8, .94),
           BGBorderColor=(.8, .3, 0, 1),
           separatorColor=(0, 0, 0, 1),
           frameColorHover=(.3, .3, .3, 1),
           frameColorPress=(0, 1, 0, .85),
           textColorReady=(0, 0, 0, 1),
           textColorHover=(1, .7, .2, 1),
           textColorPress=(0, 0, 0, 1),
           textColorDisabled=(.65, .65, .65, 1),
        )

        # CrazyFlie
        self.crazyLoggers = []
        if configurations['autoconnect']:
            self.crazyFlieConnection()

    def createMenuItems(self):
        return (
            ('Connect Crazyflies', 0, self.crazyFlieConnection),
            0,  # separator
            ('Exit > Escape', 0, self.closeAll),
        )

    def createHelpMenuItems(self):
        return (
            ('User guide', 0, self.userGuideDialog),
            0,  # separator
            ('About', 0, self.aboutDialog),
        )

    def userGuideDialog(self):
        # create a frame
        l = -0.9
        r = 0.9
        b = -0.5
        t = 0.5
        self.popupFrame = DirectFrame(frameColor=(.9, .9, .8, .94), frameSize=(l, r, b, t))
        self.popupFrame.setPos(0, 0, 0)
        LSborder = LineSegs()
        LSborder.setThickness(2)
        LSborder.setColor(0, 0, 0, 1)
        LSborder.moveTo(l, 0, b)
        LSborder.drawTo(r, 0, b)
        LSborder.moveTo(l, 0, b)
        LSborder.drawTo(-r, 0, -b)
        LSborder.moveTo(-l, 0, -b)
        LSborder.drawTo(r, 0, b)
        LSborder.moveTo(-l, 0, -b)
        LSborder.drawTo(-r, 0, -b)
        self.popupFrame.attachNewNode(LSborder.create())

        # Aggiungo il testo
        OnscreenText(text='User Guide\n\n'
                          'Arrow Up : Rotate around Y-axis in a clockwise direction    \n'
                          'Arrow Down : Rotate around Y-axis in a anticlockwise direction\n'
                          'Arrow Left : Rotate around X-axis in a clockwise direction    \n'
                          'Arrow Right: Rotate around X-axis in a anticlockwise direction\n'
                          '+ key : Zoom-in the interface\n'
                          '- key : Zoom-out the interface',
                      pos=(0, t/2),
                      scale=0.06,
                      parent=self.popupFrame)

        # create a button
        DirectButton(text=("Close"),
                     pos=(0, 0, -0.4),
                     scale=0.07,
                     command=self.closePopup,
                     parent=self.popupFrame)

    def aboutDialog(self):
        # create a frame
        l = -0.5
        r = 0.5
        b = -0.5
        t = 0.5
        self.popupFrame = DirectFrame(frameColor=(.9,.9,.8,.94), frameSize=(-0.5, 0.5, -0.5, 0.5))
        self.popupFrame.setPos(0, 0, 0)
        LSborder = LineSegs()
        LSborder.setThickness(2)
        LSborder.setColor(0, 0, 0, 1)
        LSborder.moveTo(l, 0, b)
        LSborder.drawTo(r, 0, b)
        LSborder.moveTo(l, 0, b)
        LSborder.drawTo(-r, 0, -b)
        LSborder.moveTo(-l, 0, -b)
        LSborder.drawTo(r, 0, b)
        LSborder.moveTo(-l, 0, -b)
        LSborder.drawTo(-r, 0, -b)
        self.popupFrame.attachNewNode(LSborder.create())

        # Aggiungo il testo
        OnscreenText(text='Object Locator v1.0\n\n'
                          'Alberto Petrucci \n'
                          'petrucci.alberto@gmail.com\n\n'
                          'Copyright - 2017',
                     pos=(0, t/2),
                     scale=0.07,
                     parent=self.popupFrame)

        # create a button
        DirectButton(text=("Close"),
                     pos=(0, 0, -0.3),
                     scale=0.07,
                     command=self.closePopup,
                     parent=self.popupFrame)

    # callback function to set  text
    def closePopup(self):
        self.popupFrame.destroy()

    def crazyFlieConnection(self):
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)
        # Scan for Crazyflies and use the first one found
        print('Scanning interfaces for Crazyflies...')
        available = cflib.crtp.scan_interfaces()
        #print('Crazyflies found:')
        #for i in available:
        #    print(i[0])

        self.set_point_node = loader.loadModel("models/sphere.egg")
        self.set_point_node.setScale(0.1, 0.1, 0.1)
        self.set_point_node.setPos(-16.5, 1.3, -7)
        self.set_point_node.setColor((0, 1, 0, 1))
        self.set_point_node.reparentTo(render)

        if len(available) > 0:
            # create a cube foreach crazyflies
            self.cubesCreation()

            # self.crazyLogger = Logger('radio://0/100/250K', 10, cube)
            i = 0
            for link in crazyfliesLink:
                logger = Logger(link,
                                NodesPositions,
                                Sequence,
                                self.cubes[i],
                                self.set_point_node,
                                land_altitude_step=configurations['land_altitude_step'],
                                show_realtime_plot=configurations['show_realtime_plots'])
                self.crazyLoggers.append(logger)
                i = i + 1
        else:
            print('No Crazyflies found!!')

    def start_sequence(self):
        print 'Start Sequence!'
        for logger in self.crazyLoggers:
            logger.start_sequence()

    def emergency_stop(self):
        print 'Emergency Stop!'
        for logger in self.crazyLoggers:
            logger.emergency_stop()

    def landing(self):
        print 'Landing!'
        for logger in self.crazyLoggers:
            logger.enable_landing()

    def cubesCreation(self):
        # Note: it isn't particularly efficient to make every face as a separate Geom.
        # instead, it would be better to create one Geom holding all of the faces.
        i = 0
        for link in crazyfliesLink:
            square0 = makeSquare(-1, -1, -1, 1, -1, 1, c=[0.0, 0.0, 1.0, 1])
            square1 = makeSquare(-1, 1, -1, 1, 1, 1, c=[0.0, 1.0, 1.0, 1])
            square2 = makeSquare(-1, 1, 1, 1, -1, 1, c=[1.0, 0.0, 0.0, 1])
            square3 = makeSquare(-1, 1, -1, 1, -1, -1, c=[1.0, 0.0, 1.0, 1])
            square4 = makeSquare(-1, -1, -1, -1, 1, 1, c=[1.0, 1.0, 0.0, 1])
            square5 = makeSquare(1, -1, -1, 1, 1, 1, c=[1.0, 1.0, 1.0, 1])
            snode = GeomNode('square_'+str(i))
            snode.addGeom(square0)
            snode.addGeom(square1)
            snode.addGeom(square2)
            snode.addGeom(square3)
            snode.addGeom(square4)
            snode.addGeom(square5)

            cube = render.attachNewNode(snode)
            cube.setPos(20*i, 20*i, 20*i)

            # OpenGl by default only draws "front faces" (polygons whose vertices are
            # specified CCW).
            cube.setTwoSided(True)
            self.cubes.append(cube)
            i = i + 1

    def setKey(self, name, value):
        if name == 'left':
            self.cameraAngles[0] -= 45
        elif name == 'right':
            self.cameraAngles[0] += 45
        elif name == 'forward':
            self.cameraAngles[1] += 30
        elif name == "backward":
            self.cameraAngles[1] -= 30
        elif name == 'top':
            self.cameraAngles[2] += 45
        elif name == "down":
            self.cameraAngles[2] -= 45

        # Traslo la telecamera principale avanti/indietro
        base.camera.posInterval(0.2, (0, self.cameraAngles[1], 0)).start()

        cameraPivot.hprInterval(0.2, (self.cameraAngles[0], self.cameraAngles[2], 0)).start()

        # Ruoto il sistema di riferimento (in baso a sx)
        self.cameraRS.hprInterval(0.2, (-self.cameraAngles[0], -self.cameraAngles[2], 0)).start()

        # Nascondo la faccia frontale
        self.grid.hideFrontFace(self.cameraAngles[2]%360, self.cameraAngles[0]%360)

    def toggleLightsSide(self):
        self.LightsOn = not self.LightsOn

        if self.LightsOn:
            render.setLight(self.slnp)
            self.slnp.setPos(base.cam, 10, -400, 0)
            self.slnp.lookAt(10, 0, 0)
        else:
            render.setLightOff(self.slnp)

    def placeNodes(self, positions):
        minX = 0
        minY = 0
        minZ = 0
        maxX = 0
        maxY = 0
        maxZ = 0
        for position in positions:
            if position[0] < minX:
                minX = position[0]
            if position[1] < minY:
                minY = position[1]
            if position[2] < minZ:
                minZ = position[2]

            if position[0] > maxX:
                maxX = position[0]
            if position[1] > maxY:
                maxY = position[1]
            if position[2] > maxZ:
                maxZ = position[2]
        for position in positions:
            #position[0] = ((position[0]-1.235)/2.5725 - 1)*63 + 1.3
            #position[1] = ((position[1]-0.045)/1.739 - 1)*63 - 16.5
            #position[2] = ((position[2]-1.128)/0.7155 - 1)*63 - 7

            positionX = ((position[0]-minX)/(maxX/2) - 1)*63 - 16.5
            positionY = ((position[1]-minY)/(maxY/2) - 1)*63 + 1.3
            positionZ = ((position[2]-minZ)/(maxZ/2) - 1)*63 - 7

            node = loader.loadModel("models/sphere.egg")
            node.setScale(0.1, 0.1, 0.1)
            node.setPos(positionX, positionY, positionZ)
            node.setColor((1, 0, 0, 1))
            node.reparentTo(render)

    def closeAll(self):
        for logger in self.crazyLoggers:
            logger.closeAll()
        sys.exit()

t = MyTapper()
base.run()
