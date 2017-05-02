#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
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
Used for sending control setpoints to the Crazyflie
"""
from __future__ import division
import struct
import numpy as np
import binascii
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

TYPE_STOP = 0
TYPE_VELOCITY_WORLD = 1

from math import copysign, frexp, isinf, isnan, trunc

NEGATIVE_INFINITY = b'\x00\xfc'
POSITIVE_INFINITY = b'\x00\x7c'
POSITIVE_ZERO = b'\x00\x00'
NEGATIVE_ZERO = b'\x00\x80'
# exp=2**5-1 and significand non-zero
EXAMPLE_NAN = struct.pack('<H', (0b11111 << 10) | 1)


class Commander():

    def binary16(self, f):
        """Convert Python float to IEEE 754-2008 (binary16) format.
        https://en.wikipedia.org/wiki/Half-precision_floating-point_format
        """
        if isnan(f):
            return EXAMPLE_NAN

        sign = copysign(1, f) < 0
        if isinf(f):
            return NEGATIVE_INFINITY if sign else POSITIVE_INFINITY

        # 1bit        10bits             5bits
        # f = (-1)**sign * (1 + f16 / 2**10) * 2**(e16 - 15)
        # f = (m * 2)                        * 2**(e - 1)
        m, e = frexp(f)
        assert not (isnan(m) or isinf(m))
        if e == 0 and m == 0:  # zero
            return NEGATIVE_ZERO if sign else POSITIVE_ZERO

        f16 = trunc((2 * abs(m) - 1) * 2 ** 10)  # XXX round toward zero
        assert 0 <= f16 < 2 ** 10
        e16 = e + 14
        if e16 <= 0:  # subnormal
            # f = (-1)**sign * fraction / 2**10 * 2**(-14)
            f16 = int(2 ** 14 * 2 ** 10 * abs(f) + .5)  # XXX round
            e16 = 0
        elif e16 >= 0b11111:  # infinite
            return NEGATIVE_INFINITY if sign else POSITIVE_INFINITY
        else:
            # normalized value
            assert 0b00001 <= e16 < 0b11111, (f, sign, e16, f16)
        """
        http://blogs.perl.org/users/rurban/2012/09/reading-binary-floating-point-numbers-numbers-part2.html
        sign    1 bit  15
        exp     5 bits 14-10     bias 15
        frac   10 bits 9-0
        (-1)**sign * (1 + fraction / 2**10) * 2**(exp - 15)
        +-+-----[1]+----------[0]+ # little endian
        |S| exp    |    fraction |
        +-+--------+-------------+
        |1|<---5-->|<---10bits-->|
        <--------16 bits--------->
        """
        return struct.pack('<H', (sign << 15) | (e16 << 10) | f16)

    def _float_from_unsigned16(self, n):
        assert 0 <= n < 2 ** 16
        sign = n >> 15
        exp = (n >> 10) & 0b011111
        fraction = n & (2 ** 10 - 1)
        if exp == 0:
            if fraction == 0:
                return -0.0 if sign else 0.0
            else:
                return (-1) ** sign * fraction / 2 ** 10 * 2 ** (-14)  # subnormal
        elif exp == 0b11111:
            if fraction == 0:
                return float('-inf') if sign else float('inf')
            else:
                return float('nan')
        return (-1) ** sign * (1 + fraction / 2 ** 10) * 2 ** (exp - 15)


    """
    Used for sending control setpoints to the Crazyflie
    """

    def __init__(self, crazyflie=None):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        self._cf = crazyflie
        self._x_mode = False

    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled

    def send_setpoint(self, roll, pitch, yaw, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw/thrust to the copter

        The arguments roll/pitch/yaw/trust is the new setpoints that should
        be sent to the copter
        """
        if thrust > 0xFFFF or thrust < 0:
            raise ValueError('Thrust must be between 0 and 0xFFFF')

        if self._x_mode:
            roll, pitch = 0.707 * (roll - pitch), 0.707 * (roll + pitch)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffH', roll, -pitch, yaw, thrust)
        self._cf.send_packet(pk)

    def send_stop_setpoint(self):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<B', TYPE_STOP)
        self._cf.send_packet(pk)

    def send_backstepping_control_references(self, enable,
                                             xpos, xvel, xacc,
                                             ypos, yvel, yacc,
                                             zpos, zvel, zacc):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_BACKSTEPPING
        pk.data = struct.pack('<hhhhhhhhhH', 1000*xpos, 1000*xvel, 1000*xacc, 1000*ypos, 1000*yvel, 1000*yacc, 1000*zpos, 1000*zvel, 1000*zacc, enable)
        self._cf.send_packet(pk)

    def send_control_references(self, xpos, xvel, xacc,
                                      ypos, yvel, yacc,
                                      zpos, zvel, zacc,
                                      yawpos, yawvel):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        #  CONTROLMODE_ACCELERATION(mode) ((0b001 & mode) != 0)
        #  CONTROLMODE_VELOCITY(mode)     ((0b010 & mode) != 0)
        #  CONTROLMODE_POSITION(mode)     ((0b100 & mode) != 0)
        packetHasExternalReference = 0
        setEmergency = 0
        resetEmergency = 1
        xmode = 4  # position-control
        ymode = 4  # position-control
        zmode = 4  # position-control
        #header = int(packetHasExternalReference+setEmergency+resetEmergency+xmode+ymode+zmode+'0000', 2)
        #print packetHasExternalReference+setEmergency+resetEmergency+xmode+ymode+zmode+'0000'
        '''
        xpos = int(binascii.hexlify(self.binary16(xpos)), 16)
        print binascii.hexlify(self.binary16(xpos)), xpos
        xvel = int(binascii.hexlify(self.binary16(xvel)), 16)
        xacc = int(binascii.hexlify(self.binary16(xacc)), 16)

        ypos = int(binascii.hexlify(self.binary16(ypos)), 16)
        yvel = int(binascii.hexlify(self.binary16(yvel)), 16)
        yacc = int(binascii.hexlify(self.binary16(yacc)), 16)

        zpos = int(binascii.hexlify(self.binary16(zpos)), 16)
        zvel = int(binascii.hexlify(self.binary16(zvel)), 16)
        zacc = int(binascii.hexlify(self.binary16(zacc)), 16)

        yawpos = int(binascii.hexlify(self.binary16(yawpos)), 16)
        yawvel = int(binascii.hexlify(self.binary16(yawvel)), 16)
        '''
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BBBBBHHHHHHHHHHH', setEmergency, resetEmergency, xmode, ymode, zmode, 1000*xpos, 1000*xvel, 1000*xacc, 1000*ypos, 1000*yvel, 1000*yacc, 1000*zpos, 1000*zvel, 1000*zacc, 1000*yawpos, 1000*yawvel)
        self._cf.send_packet(pk)



    def send_control_reset(self):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        #  CONTROLMODE_ACCELERATION(mode) ((0b001 & mode) != 0)
        #  CONTROLMODE_VELOCITY(mode)     ((0b010 & mode) != 0)
        #  CONTROLMODE_POSITION(mode)     ((0b100 & mode) != 0)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BBBBBHHHHHHHHHHH', 0, 1, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self._cf.send_packet(pk)

    def send_control_emergency_stop(self):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        #  CONTROLMODE_ACCELERATION(mode) ((0b001 & mode) != 0)
        #  CONTROLMODE_VELOCITY(mode)     ((0b010 & mode) != 0)
        #  CONTROLMODE_POSITION(mode)     ((0b100 & mode) != 0)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BBBBBHHHHHHHHHHH', 1, 0, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self._cf.send_packet(pk)

    def send_velocity_world_setpoint(self, vx, vy, vz, yawrate):
        """
        Send Velocity in the world frame of reference setpoint.

        vx, vy, vz are in m/s
        yawrate is in rad/s
        """
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<Bffff', TYPE_VELOCITY_WORLD,
                              vx, vy, vz, yawrate)
        self._cf.send_packet(pk)
