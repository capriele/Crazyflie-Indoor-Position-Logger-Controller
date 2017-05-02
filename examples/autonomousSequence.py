# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
from __future__ import division
import logging
import threading

import cflib.crtp  # noqa
from drone_quaternion import Quadcopter
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Keypress
import pygame, sys, time
from pygame.locals import *
SCREEN_WIDTH = 400
SCREEN_HEIGHT = 310
FONT_SIZE = 30

from numpy import *
from math import *

stop = False
logger = False
crazyflie = False
positionIndex = 0

ONLINE_CONTROLLER = True
MANUAL_TRAJECTORY = False

# Variabili atterraggio
land = False
land_altitude = False  # 10cm
land_altitude_step = 0.1  # 10cm

uri = 'radio://0/120/2M'
# uri = 'radio://0/40/250K'

x = 1.5
y = 1.4
z = 1.6
#            x  y   z  YAW
sequence = [(x, y, z, 0),
            (1.5*x, y, z, 0),
            (1.5*x, 0.5*y, z, 0),
            (x, 0.5*y, z, 0),
            (x, y, z, 0),
            (x, y, 0.5, 0)
            ]
# Manual Trajectory
trajectoryStep = 0.10  # 10cm
manual = {'x': x, 'y': y, 'z': z}

positionLogger = False
quadcopter = Quadcopter(0.01)


def start_sequence(cf):
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if keys[K_RETURN]:
        print 'Sequence Started!'
        cf.commander.send_setpoint(0, 0, 0, 0)
        #for i in range(0, 500):
        #cf.param.set_value('lqrCtrl.tM', '{}'.format(1.01))
        #cf.param.set_value('lqrCtrl.rM', '{}'.format(-1))
        #cf.param.set_value('lqrCtrl.pM', '{}'.format(-1))
        # cf.param.set_value('flightmode.posSet', '1')
        # cf.param.set_value('flightmode.poshold', '1')
        return True
    return False


def emergency_stop(cf):
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if keys[K_ESCAPE]:
        print 'Emergency Stop!'
        cf.commander.send_setpoint(0, 0, 0, 0)
        #for i in range(0, 500):
        #cf.param.set_value('lqrCtrl.tM', '{}'.format(1.01))
        #cf.param.set_value('lqrCtrl.rM', '{}'.format(-1))
        #cf.param.set_value('lqrCtrl.pM', '{}'.format(-1))
        # cf.param.set_value('flightmode.posSet', '1')
        # cf.param.set_value('flightmode.poshold', '1')
        return True
    return False


def text_to_screen(screen, text, x, y, size=50, color=(200, 000, 000)):
    try:
        basicfont = pygame.font.SysFont(None, size)
        text = basicfont.render(text, True, color, (255, 255, 255))
        text_rect = text.get_rect(center=(SCREEN_WIDTH / 2, y+10))
        screen.blit(text, text_rect)
        pygame.display.update()
    except Exception, e:
        print 'Font Error, saw it coming'
        raise e


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, maxLen):

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
        self.maxLen = maxLen

        self.data = []

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)
        # Scan for Crazyflies and use the first one found
        print('Scanning interfaces for Crazyflies...')
        available = cflib.crtp.scan_interfaces()
        print('Crazyflies found:')
        for i in available:
            print i[0]

        if len(available) <= 0:
            print('No Crazyflies found, cannot run example')

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
        self.status = False
        self.landing = False

    def crazyflie(self):
        return self._cf

    def close_link(self):
        self._cf.close_link()

    def getData(self):
        return self.data

    def log_status(self):
        return self.status

    def state(self):
        return self.q0, self.q1, self.q2, self.q3, self.wx, self.wy, self.wz, self.px, self.py, self.pz, self.vx, self.vy, self.vz

    def getLanding(self):
        return self.landing

    def enable_landing(self):
        self.landing = True

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
        #self.log_config3.add_variable('state.px', 'float')
        #self.log_config3.add_variable('state.py', 'float')
        #self.log_config3.add_variable('state.pz', 'float')
        #self.log_config3.add_variable('state.vx', 'float')
        #self.log_config3.add_variable('state.vy', 'float')
        #self.log_config3.add_variable('state.vz', 'float')
        #self.log_config3.add_variable('ctrltarget.px', 'float')
        #self.log_config3.add_variable('ctrltarget.py', 'float')
        #self.log_config3.add_variable('ctrltarget.pz', 'float')
        #self.log_config3.add_variable('ctrltarget.vx', 'float')
        #self.log_config3.add_variable('ctrltarget.vy', 'float')
        #self.log_config3.add_variable('ctrltarget.vz', 'float')
        #self.log_config3.add_variable('ctrltarget.ax', 'float')
        #self.log_config3.add_variable('ctrltarget.ay', 'float')
        #self.log_config3.add_variable('ctrltarget.az', 'float')
        self.log_config3.add_variable('Bks.dx', 'float')
        self.log_config3.add_variable('Bks.dy', 'float')
        self.log_config3.add_variable('Bks.dz', 'float')
        self.log_config3.add_variable('Bks.dwx', 'float')
        self.log_config3.add_variable('Bks.dwy', 'float')
        self.log_config3.add_variable('Bks.dwz', 'float')

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
            if not ONLINE_CONTROLLER:
                self.log_config1.start()
                self.log_config2.start()
            self.log_config3.start()
            print "Log succesfully started!"
            self.status = True
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _log_data_quaternion(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #self.plot.drawNow(np.random.random(self.plot.X.shape))
        self.q0 = data['state.q0']
        self.q1 = data['state.q1']
        self.q2 = data['state.q2']
        self.q3 = data['state.q3']

    def _log_data_angular(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        self.wx = data['state.wx']
        self.wy = data['state.wy']
        self.wz = data['state.wz']

    def _log_data_linear(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #self.px = data['state.px']
        #self.py = data['state.py']
        #self.pz = data['state.pz']
        #self.vx = data['state.vx']
        #self.vy = data['state.vy']
        #self.vz = data['state.vz']
        self.data = data
        print data

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))
        self._cf.open_link(link_uri)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

# Control main loop


def control(interval, function, iterations=0):
    global stop
    if not stop:
        if iterations != 1:
            threading.Timer(interval, control, [interval, function, 0 if iterations == 0 else iterations-1]).start()
        function()


t = 0
X = False
Xd = 0
Xdd = 0
Y = False
Yd = 0
Ydd = 0
step = 0
def set_point():
    global positionIndex
    global sequence
    global quadcopter
    global crazyflie
    global logger
    global stop
    global land
    global land_altitude
    global land_altitude_step
    global ONLINE_CONTROLLER
    global t
    global X
    global Xd
    global Xdd
    global Y
    global Yd
    global Ydd
    global step
    if not logger.getLanding():
        if not stop:
            position = sequence[0]
            '''
            if not X:
                X = position[0]
            if not Y:
                Y = position[0]

            if t >= 10:
                step = step + 1
                if step == 3:
                    step = 0
                t = 0

            dt = 0.0001
            if step == 0:
                #Traiettoria X
                if X < position[0]*1.5:
                    X = X + dt
                    Xd = dt
                    Xdd = 0
                else:
                    X = position[0] * 1.5
                    Xd = 0

                # Traiettoria Y
                if Y > position[1] * 3 / 4:
                    Y = Y - dt
                    Yd = -dt
                    Ydd = 0
                else:
                    Y = position[1] * 3 / 4
                    Yd = 0
            elif step == 1:
                if X > position[0]:
                    X = X - dt
                    Xd = -dt
                    Xdd = 0
                else:
                    X = position[0]
                    Xd = 0

                if Y >= position[1] * 0.5:
                    Y = Y - dt
                    Yd = -dt
                    Ydd = 0
                else:
                    Y = position[1] * 0.5
                    Yd = 0
            else:
                X = position[0]
                Xd = 0
                Y = Y + dt
                Yd = dt
                Ydd = 0
                if Y > position[1]:
                    Y = position[1]
                    Yd = 0
            '''

            zd = t
            zdd = 1
            if zd > position[2]:
                zd = position[2]
                zdd = 0

            w = 3#3#4
            a = 0.4#0.4
            xd = matrix([
                # Roll
                [0, 0, 0],
                # Pitch
                [0, 0, 0],
                # Yaw
                [0, 0, 0],
                # X
                # [self.setPoint[7, 0], 0, 0],
                [position[0] + a * sin(w * t), w * a * cos(w * t), - w * w * a * sin(w * t)],  # circle
                #[position[0] + a * sin(w * t), w * a * cos(w * t), - w * w * a * sin(w * t)],  # infty
                # [X, Xd, Xdd],  # triangle


                # Y
                # [self.setPoint[8, 0], 0, 0],
                [position[1] + a * cos(w * t) - a, - w * a * sin(w * t), - w * w * a * cos(w * t)],  # circle
                #[position[1] + a * sin(w/2 * t), w/2 * a * cos(w/2 * t), - w/2 * w/2 * a * sin(w/2 * t)],  # infty
                # [Y, Yd, Ydd],  # triangle

                # Z
                [zd, zdd, 0],
            ])
            if ONLINE_CONTROLLER:
                #crazyflie.commander.send_control_references(xd[3, 0], xd[3, 1], xd[3, 2],
                #                                            xd[4, 0], xd[4, 1], xd[4, 2],
                #                                            xd[5, 0], xd[5, 1], xd[5, 2],
                #                                            0.0, 0.0)
                crazyflie.commander.send_backstepping_control_references(1, xd[3, 0], xd[4, 0], xd[5, 0],
                                                                         xd[3, 1], xd[4, 1], xd[5, 1],
                                                                         xd[3, 2], xd[4, 2], xd[5, 2])
            else:
                quadcopter.setBacksteppingSetPoint(xd)
            t += 0.03
            #print t
        else:
            crazyflie.commander.send_setpoint(0, 0, 0, 0)
            crazyflie.commander.send_stop_setpoint()
    else:
        # rimango nella posizione attuale ed piano piano diminuisco la z
        position = sequence[0]
        xd = matrix([
            # Roll
            [0, 0, 0],
            # Pitch
            [0, 0, 0],
            # Yaw
            [0, 0, 0],
            # X
            [position[0], 0, 0],
            # Y
            [position[1], 0, 0],
            # Z
            [land_altitude, 0, 0],
        ])
        if ONLINE_CONTROLLER:
            #crazyflie.commander.send_control_references(xd[3, 0], xd[3, 1], xd[3, 2],
            #                                            xd[4, 0], xd[4, 1], xd[4, 2],
            #                                            xd[5, 0], xd[5, 1], xd[5, 2],
            #                                            0, 0)
            crazyflie.commander.send_backstepping_control_references(1, xd[3, 0], xd[4, 0], xd[5, 0], 0, 0, 0, 0, 0, 0)
        else:
            quadcopter.setBacksteppingSetPoint(xd)
        land_altitude = land_altitude - land_altitude_step * 0.02
        if land_altitude < 0:
            land_altitude = 0
            crazyflie.commander.send_backstepping_control_references(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        print land_altitude
    '''
        if positionIndex < len(sequence):
            position = sequence[positionIndex]
            print('Setting position {}'.format(position))
            quadcopter.setSetPoint(1, 0, 0, 0, 0, 0, 0, position[0], position[1], position[2], 0, 0, 0)
        else:
            stop = True
            crazyflie.commander.send_setpoint(0, 0, 0, 0)
            logger.close_link()
            exit(0)
        positionIndex = positionIndex + 1
    else:
        # rimango nella posizione attuale ed piano piano diminuisco la z
        position = sequence[positionIndex]
        if not land_altitude:
            land_altitude = position[2]
        quadcopter.setSetPoint(1, 0, 0, 0, 0, 0, 0, position[0], position[1], land_altitude, 0, 0, 0)
        land_altitude = land_altitude - land_altitude_step*0.01
        print land_altitude
    '''


def loop():
    global quadcopter
    global crazyflie
    global logger
    global stop
    if not stop:
        # Aggiorno lo stato del quadcopter
        # se non ricevo tutti i dati vado in predizione con il modello
        (q0, q1, q2, q3, wx, wy, wz, px, py, pz, vx, vy, vz) = logger.state()
        quadcopter.setState(q0, q1, q2, q3, wx, wy, wz, px, py, pz, vx, vy, vz)

        # LQR Control
        '''
        quadcopter.update()
        # Get motor input
        m1, m2, m3, m4 = quadcopter.getMotorInput()
        T = (m1 + m2 + m3 + m4) / 4
        R = (m4 + m3 - m2 - m1) * 8.2
        P = (m1 + m4 - m2 - m3) * 8.2
        '''

        # BackStepping Control
        quadcopter.backstepping2()

        # Get motor input
        m1, m2, m3, m4 = quadcopter.getMotorInput()

        T = (m1 + m2 + m3 + m4) / 4
        # R = -(m4 + m3 - m2 - m1) * 2.6
        # P = (m1 + m4 - m2 - m3) * 2.6
        R = (m4 + m3 - m2 - m1)
        P = (m1 + m4 - m2 - m3)
        Y = (m1 - m2 + m3 - m4)/4

        if T > 65535:
            T = 65535
        print R, P, T
        # print px, py, pz
        # Roll > 0 => diminuisce la y
        # Roll < 0 => aumenta la y
        # Pitch > 0 => aumenta la x
        # Pitch < 0 => diminuisce la x
        crazyflie.commander.send_setpoint(R, P, Y, int(T))
    else:
        crazyflie.commander.send_setpoint(0, 0, 0, 0)


if __name__ == '__main__':

    # Initialize connection
    logger = LoggingExample(uri, 10)
    while not logger.log_status():
        continue
    crazyflie = logger.crazyflie()
    if ONLINE_CONTROLLER:
        crazyflie.param.set_value('flightmode.posSet', '1')
        # disturbace estimation gains
        #LEVANTE
        #crazyflie.param.set_value('posCtlBks.alpha', '0.9999')
        #crazyflie.param.set_value('posCtlBks.alphaw', '0.999')
        #crazyflie.param.set_value('posCtlBks.L1', '10')
        #crazyflie.param.set_value('posCtlBks.L2', '20')

        #OBSERVER
        #crazyflie.param.set_value('posCtlBks.alpha', '0.00001')
        #crazyflie.param.set_value('posCtlBks.L1', '100')
        #crazyflie.param.set_value('posCtlBks.L2', '20000')
        ##crazyflie.param.set_value('posCtlBks.alpha', '0.001')
        ##crazyflie.param.set_value('posCtlBks.L1', '1')
        ##crazyflie.param.set_value('posCtlBks.L2', '1000')
        crazyflie.param.set_value('posCtlBks.alpha', '0.1')
        crazyflie.param.set_value('posCtlBks.L1', '0.1')
        crazyflie.param.set_value('posCtlBks.L2', '0.1')

        # x-gains
        crazyflie.param.set_value('posCtlBks.kx1', '2')
        crazyflie.param.set_value('posCtlBks.kx2', '1')

        # y-gains
        crazyflie.param.set_value('posCtlBks.ky1', '2')
        crazyflie.param.set_value('posCtlBks.ky2', '1')

        # z-gains
        crazyflie.param.set_value('posCtlBks.kz1', '5')
        crazyflie.param.set_value('posCtlBks.kz2', '2')

        # roll-gains
        crazyflie.param.set_value('posCtlBks.kr1', '20')
        crazyflie.param.set_value('posCtlBks.kr2', '6')

        # pitch-gains
        crazyflie.param.set_value('posCtlBks.kp1', '20')
        crazyflie.param.set_value('posCtlBks.kp2', '6')

        # yaw-gains
        crazyflie.param.set_value('posCtlBks.kw1', '20')
        crazyflie.param.set_value('posCtlBks.kw2', '6')

        crazyflie.param.set_value('posCtlBks.errorLimit', '100000')

    pygame.init()
    pygame.font.init()
    pygame.joystick.init()
    #joystick = pygame.joystick.Joystick(0)
    #joystick.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    screen.fill((255, 255, 255))
    pygame.display.set_caption('Hello World')
    #numaxes = joystick.get_numaxes()
    loopQuit = False

    #Print commands on the screen
    text_to_screen(screen, 'Manual Mode Commands', 10, 10, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, 'Arrow Up: X +', 10, 10+FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, 'Arrow Down: X -', 10, 10+2*FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, 'Arrow Left: Y +', 10, 10+3*FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, 'Arrow Right: Y -', 10, 10+4*FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, '[U]p: Z +', 10, 10+5*FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, '[D]own: Z -', 10, 10+6*FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, '[L]: Landing', 10, 10+7*FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, '[ESC]: Emergency Stop', 10, 10+8*FONT_SIZE, FONT_SIZE, (200, 0, 0))
    text_to_screen(screen, '[ENTER]: Start', 10, 10+9*FONT_SIZE, FONT_SIZE, (200, 0, 0))

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    # initialize figure
    print "Press ENTER to start"
    while not start_sequence(crazyflie):
        continue

    crazyflie.commander.send_control_reset()

    # Suppongo che il drone parte dalle coordinate (x,y) corrette
    quadcopter.setState(1, 0, 0, 0, 0, 0, 0, sequence[0][0], sequence[0][1], sequence[0][2], 0, 0, 0)

    # call loop at 100Hz forever
    #print "Start Control Loop!"
    if not ONLINE_CONTROLLER:
        control(0.01, loop)

    # cambio il set_point ogni tot. secondi
    if not MANUAL_TRAJECTORY:
        control(0.01*3, set_point)

    # Posizioni iniziali
    deltaP = 0.005
    positionX = sequence[0][0]
    positionY = sequence[0][1]
    positionZ = sequence[0][2]
    quadcopter.setSetPoint(1, 0, 0, 0, 0, 0, 0, positionX, positionY, positionZ, 0, 0, 0)

    buttonPressed = False
    while not loopQuit:
        # Leggo il JoyStick
        #axisX = -round(joystick.get_axis(1), 1)  # get the analog value of the specified axis
        #axisY = -round(joystick.get_axis(0), 1)  # get the analog value of the specified axis
        '''
        axisZ = -round(joystick.get_axis(3), 1)  # get the analog value of the specified axis
        button4 = joystick.get_button(4)  # get the analog value of the specified axis
        button5 = joystick.get_button(5)  # get the analog value of the specified axis
        button6 = joystick.get_button(6)  # get the analog value of the specified axis
        button7 = joystick.get_button(7)  # get the analog value of the specified axis

        # Aggiorno il setpoint
        if button4 is not 0:
            positionX = positionX + deltaP
        if button6 is not 0:
            positionX = positionX - deltaP
        if button5 is not 0:
            positionY = positionY - deltaP
        if button7 is not 0:
            positionY = positionY + deltaP
        if axisZ is not 0.0:
            positionZ = positionZ + 0.01*axisZ
        '''
        #quadcopter.setSetPoint(1, 0, 0, 0, 0, 0, 0, positionX+sin(w*t), positionY+cos(w*t), positionZ, w*cos(w*t), -w*sin(w*t), 0)
        #print positionX, positionY, positionZ

        pygame.event.pump()
        keys = pygame.key.get_pressed()
        if keys[K_l]:
            # land
            print 'Landing!'
            land_altitude = positionZ
            logger.enable_landing()
            land = True
        elif keys[K_UP]:
            # increase X
            if not buttonPressed:
                manual['x'] = manual['x'] + trajectoryStep
            #buttonPressed = True
        elif keys[K_DOWN]:
            # decrease X
            if not buttonPressed:
                manual['x'] = manual['x'] - trajectoryStep
            #buttonPressed = True
        elif keys[K_LEFT]:
            # increase Y
            if not buttonPressed:
                manual['y'] = manual['y'] + trajectoryStep
            #buttonPressed = True
        elif keys[K_RIGHT]:
            # decrease Y
            if not buttonPressed:
                manual['y'] = manual['y'] - trajectoryStep
            #buttonPressed = True
        elif keys[K_u]:
            # increase Z
            if not buttonPressed:
                manual['z'] = manual['z'] + trajectoryStep
            #buttonPressed = True
        elif keys[K_d]:
            # decrease Z
            if not buttonPressed:
                manual['z'] = manual['z'] - trajectoryStep
            #buttonPressed = True
        elif keys[K_ESCAPE]:
            print 'Emergency Stop!'
            stop = True
            loopQuit = True
            crazyflie.commander.send_backstepping_control_references(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            crazyflie.commander.send_backstepping_control_references(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            crazyflie.commander.send_backstepping_control_references(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            crazyflie.commander.send_backstepping_control_references(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            crazyflie.commander.send_backstepping_control_references(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            crazyflie.commander.send_stop_setpoint()
            #crazyflie.commander.send_control_emergency_stop()
            logger.close_link()
        else:
            buttonPressed = False

        if MANUAL_TRAJECTORY and not stop:
            print manual
            v = 0
            a = 0
            z = manual['z']
            if z > t:
                z = t
            crazyflie.commander.send_backstepping_control_references(1, manual['x'], manual['y'], z, v, v, v, a, a, a)

        # Gestisco tutti gli eventi di PyGame
        for event in pygame.event.get():
            if event.type == QUIT:
                loopQuit = True

        if MANUAL_TRAJECTORY:
            time.sleep(0.03)
            t += 0.03
        else:
            time.sleep(0.1)
    pygame.quit()
    sys.exit()


