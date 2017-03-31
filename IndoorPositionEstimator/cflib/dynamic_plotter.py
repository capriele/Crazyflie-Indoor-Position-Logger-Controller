#
# Author: Alberto Petrucci (petrucci.alberto@gmail.com) 2017
#
#__author__ = "Alberto Petrucci"
#__copyright__ = "Copyright 2017, Alberto Petrucci"
#__credits__ = ["Alberto Petrucci"]
#__license__ = "MIT"
#__version__ = "1.0.0"
#__maintainer__ = "Alberto Petrucci"
#__email__ = "petrucci.alberto@.com"
#__status__ = "Production"
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

import numpy as np


class DynamicPlotterAttitude:

    def __init__(self, title='Dynamic Plotting with PyQtGraph', sampleinterval=0.1, timewindow=10.0, size=(600, 350), yRange=[-180,180]):
        # Data stuff
        self._interval = int(sampleinterval*1000)
        self._bufsize = int(timewindow/sampleinterval)

        self.x = np.linspace(0.0, timewindow, self._bufsize)
        self.roll = np.zeros(self._bufsize, dtype=np.float)
        self.pitch = np.zeros(self._bufsize, dtype=np.float)
        self.yaw = np.zeros(self._bufsize, dtype=np.float)

        # PyQtGraph stuff
        self.app = QtGui.QApplication([])
        self.plt = pg.plot(title=title)
        self.plt.resize(*size)
        self.plt.showGrid(x=True, y=True)
        self.plt.setRange(yRange=yRange)
        #self.plt.setLabel('left', title.lower(), '')
        self.plt.setLabel('bottom', 'time', 's')

        self.curveRoll = self.plt.plot(self.x, self.roll, pen=(255, 0, 0), name="Roll")
        self.curvePitch = self.plt.plot(self.x, self.pitch, pen=(0, 255, 0), name="Pitch")
        self.curveYaw = self.plt.plot(self.x, self.yaw, pen=(0, 0, 255), name="Yaw")

        # add legend
        self.plt.addLegend()
        self.plt.addItem(self.curveRoll, "Roll")
        self.plt.addItem(self.curvePitch, "Pitch")
        self.plt.addItem(self.curveYaw, "Yaw")

        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)

    def setRoll(self, data):
        self.roll[:-1] = self.roll[1:]
        self.roll[-1:] = data

    def setPitch(self, data):
        self.pitch[:-1] = self.pitch[1:]
        self.pitch[-1:] = data

    def setYaw(self, data):
        self.yaw[:-1] = self.yaw[1:]
        self.yaw[-1:] = data

    def updateplot(self):
        self.curveRoll.setData(self.x, self.roll)
        self.curvePitch.setData(self.x, self.pitch)
        self.curveYaw.setData(self.x, self.yaw)
        self.app.processEvents()

    def run(self):
        self.app.exec_()


class DynamicPlotterPosition:

    def __init__(self, title='Dynamic Plotting with PyQtGraph', sampleinterval=0.1, timewindow=10.0, size=(600, 350), yRange=[-180,180]):
        # Data stuff
        self._interval = int(sampleinterval*1000)
        self._bufsize = int(timewindow/sampleinterval)

        self.t = np.linspace(0.0, timewindow, self._bufsize)
        self.X = np.zeros(self._bufsize, dtype=np.float)
        self.Y = np.zeros(self._bufsize, dtype=np.float)
        self.Z = np.zeros(self._bufsize, dtype=np.float)

        # PyQtGraph stuff
        self.app = QtGui.QApplication([])
        self.plt = pg.plot(title=title)
        self.plt.resize(*size)
        self.plt.showGrid(x=True, y=True)
        self.plt.setRange(yRange=yRange)
        #self.plt.setLabel('left', title.lower(), '')
        self.plt.setLabel('bottom', 'time', 's')

        self.curveX = self.plt.plot(self.t, self.X, pen=(255, 0, 0), name="X")
        self.curveY = self.plt.plot(self.t, self.Y, pen=(0, 255, 0), name="Y")
        self.curveZ = self.plt.plot(self.t, self.Z, pen=(0, 0, 255), name="Z")

        # add legend
        self.plt.addLegend()
        self.plt.addItem(self.curveX, "X")
        self.plt.addItem(self.curveY, "Y")
        self.plt.addItem(self.curveZ, "Z")

        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)

        self.data = 0

    def setX(self, data):
        self.X[:-1] = self.X[1:]
        self.X[-1:] = data

    def setY(self, data):
        self.Y[:-1] = self.Y[1:]
        self.Y[-1:] = data

    def setZ(self, data):
        self.Z[:-1] = self.Z[1:]
        self.Z[-1:] = data

    def updateplot(self):
        self.curveX.setData(self.t, self.X)
        self.curveY.setData(self.t, self.Y)
        self.curveZ.setData(self.t, self.Z)
        self.app.processEvents()

    def run(self):
        self.app.exec_()
