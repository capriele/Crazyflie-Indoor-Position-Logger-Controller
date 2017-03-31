# -*- coding: utf-8 -*-
#
# Author: Alberto Petrucci (petrucci.alberto@gmail.com) 2017

"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import math
import time
from threading import Thread, Timer

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie

# Model
from drone_quaternion import Quadcopter

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import numpy as np

# REAL TIME GRAPH
from dynamic_plotter import DynamicPlotterAttitude, DynamicPlotterPosition


class Logger:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, nodes, sequence, cube, set_point_node, land_altitude_step=0.1, show_realtime_plot=1):
        """ Initialize and run the example with the specified link_uri """
        # Quadcopter Model
        self.quadcopter = Quadcopter(0.01)

        # The list of points that the quadcopter will follow
        self.sequence = sequence
        self.sequence_index = 0
        self.set_point_node = set_point_node

        #  Quadcopter State
        # Attitude
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0

        # Linear
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Other Params (useful for testing)
        self.status = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.mx = 0.0
        self.my = 0.0
        self.mz = 0.0

        # Nodes Positions
        self.nodes = nodes

        # Normalize Position
        self.minX = 0
        self.minY = 0
        self.minZ = 0
        self.maxX = 0
        self.maxY = 0
        self.maxZ = 0
        for position in nodes:
            if position[0] < self.minX:
                self.minX = position[0]
            if position[1] < self.minY:
                self.minY = position[1]
            if position[2] < self.minZ:
                self.minZ = position[2]
            if position[0] > self.maxX:
                self.maxX = position[0]
            if position[1] > self.maxY:
                self.maxY = position[1]
            if position[2] > self.maxZ:
                self.maxZ = position[2]

        self.cube = cube

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.show_realtime_plot = show_realtime_plot
        if self.show_realtime_plot:
            self.plotAttitude = DynamicPlotterAttitude(title='Attitude',
                                                       sampleinterval=0.1,
                                                       timewindow=10.,
                                                       size=(600, 350),
                                                       yRange=[-180, 180])
            self.plotPosition = DynamicPlotterPosition(title='Position',
                                                       sampleinterval=0.1,
                                                       timewindow=10.,
                                                       size=(600, 350),
                                                       yRange=[-180, 180])

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
        self.connected = False
        self.closed = False
        self.status = False
        self.canStart = False
        self.landing = False
        self.land_altitude_step = land_altitude_step
        self.land_altitude = 0

    def start_sequence(self):
        self.canStart = True
        self.sequence_index = 0
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

    def emergency_stop(self):
        self.canStart = False
        print 'Emergency Stop!'
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self.closeAll()

    def enable_landing(self):
        self.landing = True

    def closeAll(self):
        self.closed = True
        self.canStart = False
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self._cf.close_link()

    def state(self):
        return self.q0, self.q1, self.q2, self.q3, self.wx, self.wy, self.wz, self.px, self.py, self.pz, self.vx, self.vy, self.vz

    def getAttitude(self):
            return [self.roll, self.pitch, self.yaw]

    def getPosition(self):
        return [self.x, self.y, self.z]

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        self.log_config1 = LogConfig(name='StateAngular1', period_in_ms=10)
        self.log_config1.add_variable('state.q0', 'float')
        self.log_config1.add_variable('state.q1', 'float')
        self.log_config1.add_variable('state.q2', 'float')
        self.log_config1.add_variable('state.q3', 'float')

        self.log_config2 = LogConfig(name='StateAngular2', period_in_ms=10)
        self.log_config2.add_variable('state.wx', 'float')
        self.log_config2.add_variable('state.wy', 'float')
        self.log_config2.add_variable('state.wz', 'float')

        self.log_config3 = LogConfig(name='StateLinear', period_in_ms=10)
        self.log_config3.add_variable('state.px', 'float')
        self.log_config3.add_variable('state.py', 'float')
        self.log_config3.add_variable('state.pz', 'float')
        self.log_config3.add_variable('state.vx', 'float')
        self.log_config3.add_variable('state.vy', 'float')
        self.log_config3.add_variable('state.vz', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self.log_config1)
            self._cf.log.add_config(self.log_config2)
            self._cf.log.add_config(self.log_config3)

            # This callback will receive the data
            self.log_config1.data_received_cb.add_callback(self._log_data_quaternion)
            self.log_config2.data_received_cb.add_callback(self._log_data_angular)
            self.log_config3.data_received_cb.add_callback(self._log_data_linear)

            # This callback will be called on errors
            self.log_config1.error_cb.add_callback(self._log_error)
            self.log_config2.error_cb.add_callback(self._log_error)
            self.log_config3.error_cb.add_callback(self._log_error)

            # Start the logging
            self.log_config1.start()
            self.log_config2.start()
            self.log_config3.start()
            self.status = True
            print "Log succesfully started!"
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        self.connected = True
        if self.show_realtime_plot:
            self.plotAttitude.run()
            self.plotPosition.run()

        # Inizialize set point update thread (dtHz)
        self.set_point_thread = Thread(target=self.set_point_update)
        self.set_point_thread.start()

        # Initialize control thread
        self.control_thread = Thread(target=self.control)
        self.control_thread.start()

    def set_point_update(self):
        while not self.closed and self.sequence_index < len(self.sequence):
            if not self.landing:
                positionx = self.sequence[self.sequence_index][0]
                positiony = self.sequence[self.sequence_index][1]
                positionz = self.sequence[self.sequence_index][2]
                self.quadcopter.setSetPoint(1, 0, 0, 0, 0, 0, 0, positionx, positiony, positionz, 0, 0, 0)
                self.setPointPosition(positionx, positiony, positionz)

                time.sleep(self.sequence[self.sequence_index][3])
                if self.canStart and self.status:
                    # Increase sequence index
                    self.sequence_index += 1
            else:
                positionx = self.sequence[self.sequence_index][0]
                positiony = self.sequence[self.sequence_index][1]
                positionz = self.sequence[self.sequence_index][2]
                if not self.land_altitude:
                    self.land_altitude = positionz
                self.quadcopter.setSetPoint(1, 0, 0, 0, 0, 0, 0, positionx, positiony, self.land_altitude, 0, 0, 0)
                self.setPointPosition(positionx, positiony, self.land_altitude)
                self.land_altitude = self.land_altitude - self.land_altitude_step
                time.sleep(self.sequence[self.sequence_index][3])

    def control(self):
        while not self.closed:
            if self.canStart and self.status:
                # Aggiorno lo stato del quadcopter
                # se non ricevo tutti i dati vado in predizione con il modello
                (q0, q1, q2, q3, wx, wy, wz, px, py, pz, vx, vy, vz) = self.state()
                self.quadcopter.setState(q0, q1, q2, q3, 0, 0, 0, px, py, pz, vx, vy, vz)
                self.quadcopter.update()
                m1, m2, m3, m4 = self.quadcopter.getMotorInput()

                t = (m1 + m2 + m3 + m4) / 4
                r = (m4 + m3 - m2 - m1) * 8.2
                p = (m1 + m4 - m2 - m3) * 8.2
                if t > 65535:
                    t = 65535
                if self.land_altitude < 0:
                    t = 0
                print r, p, t
                # print px, py, pz
                # Roll > 0 => diminuisce la y
                # Roll < 0 => aumenta la y
                # Pitch > 0 => aumenta la x
                # Pitch < 0 => diminuisce la x
                self._cf.commander.send_setpoint(r, p, 0, int(t))

    @staticmethod
    def _log_error(logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _log_data_quaternion(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        self.q0 = data['state.q0']
        self.q1 = data['state.q1']
        self.q2 = data['state.q2']
        self.q3 = data['state.q3']
        self.roll, self.pitch, self.yaw = self.quadcopter.quaternion2RPY()
        self.cube.hprInterval(0.0, (self.yaw, -self.pitch, -self.roll)).start()

    def _log_data_angular(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        self.wx = data['state.wx']
        self.wy = data['state.wy']
        self.wz = data['state.wz']

    def _log_data_linear(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        self.px = data['state.px']
        self.py = data['state.py']
        self.pz = data['state.pz']
        self.vx = data['state.vx']
        self.vy = data['state.vy']
        self.vz = data['state.vz']
        self.setCubePosition(self.px, self.py, self.pz)

    def setPointPosition(self, x, y, z):
        x = ((x - self.minX) / (self.maxX / 2) - 1) * 63 - 16.5
        y = ((y - self.minY) / (self.maxY / 2) - 1) * 63 + 1.3
        z = ((z - self.minZ) / (self.maxZ / 2) - 1) * 63 - 7
        if x > 63:
            x = 63
        if x < -63:
            x = -63
        if y > 63:
            y = 63
        if y < -63:
            y = -63
        if z > 63:
            z = 63
        if z < -63:
            z = -63
        self.set_point_node.posInterval(0, (x, y, z)).start()

    def setCubePosition(self, x, y, z):
        x = ((x - self.minX) / (self.maxX / 2) - 1) * 63
        y = ((y - self.minY) / (self.maxY / 2) - 1) * 63
        z = ((z - self.minZ) / (self.maxZ / 2) - 1) * 63
        if x > 63:
            x = 63
        if x < -63:
            x = -63
        if y > 63:
            y = 63
        if y < -63:
            y = -63
        if z > 63:
            z = 63
        if z < -63:
            z = -63
        if self.show_realtime_plot:
            self.plotPosition.setX(x)
            self.plotPosition.setY(y)
            self.plotPosition.setZ(z)
        self.cube.posInterval(0, (x, y, z)).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

if __name__ == '__main__':

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = Logger('radio://0/100/250K', 10)
    else:
        print('No Crazyflies found, cannot run example')

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    # initialize figure
    while le.is_connected:
        time.sleep(1)
