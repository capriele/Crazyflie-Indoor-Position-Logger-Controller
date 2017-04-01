#
# Author: Alberto Petrucci (petrucci.alberto@gmail.com) 2017
#
__author__ = "Alberto Petrucci"
__copyright__ = "Copyright 2017, Alberto Petrucci"
__credits__ = ["Alberto Petrucci"]
__license__ = "Apache"
__version__ = "1.0.0"
__maintainer__ = "Alberto Petrucci"
__email__ = "petrucci.alberto@gmail.com"
__status__ = "Production"

import htmlPy
import pygame, sys, time ,os
from pygame.locals import *
import multiprocessing

class Joystick(htmlPy.Object):
    def __init__(self, app):
        super(Joystick, self).__init__()
        self.app = app
        self.analog_left_x = 0
        self.analog_left_y = 0
        sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            Joystick.joystick = pygame.joystick.Joystick(0)
            Joystick.joystick.init()
            Joystick.joystick_count = pygame.joystick.get_count()
            Joystick.numaxes = Joystick.joystick.get_numaxes()
            Joystick.numbuttons = Joystick.joystick.get_numbuttons()

        # Update the button values
    def update(self):
        loopQuit = False
        button_state = [0] * self.numbuttons
        button_analog = [0] * self.numaxes
        # while loopQuit == False:
        outstr = ""

        # Start suppressing the output on stdout from Pygame
        devnull = open('/dev/null', 'w')
        oldstdout_fno = os.dup(sys.stdout.fileno())
        os.dup2(devnull.fileno(), 1)

        # Read analog values
        for i in range(0, self.numaxes):
            button_analog[i] = self.joystick.get_axis(i)

        self.a_joystick_left_x = button_analog[0]
        self.a_joystick_left_y = button_analog[1]
        self.a_joystick_right_x = button_analog[2]
        self.a_joystick_right_y = button_analog[3]

        # Read digital values
        for i in range(0, self.numbuttons):
            button_state[i] = self.joystick.get_button(i)
        self.select = button_state[0]
        self.joystick_left = button_state[1]
        self.joystick_right = button_state[2]
        self.start = button_state[3]
        self.up = button_state[4]
        self.right = button_state[5]
        self.down = button_state[6]
        self.left = button_state[7]
        self.l2 = button_state[8]
        self.r2 = button_state[9]
        self.l1 = button_state[10]
        self.r1 = button_state[11]
        self.triangle = button_state[12]
        self.circle = button_state[13]
        self.cross = button_state[14]
        self.square = button_state[15]
        self.ps = button_state[16]

        # Enable output on stdout
        os.dup2(oldstdout_fno, 1)
        os.close(oldstdout_fno)

        # refresh
        pygame.event.get()
        return button_analog

    @htmlPy.Slot()
    def read(self):
        self.update()
        w = 200
        w1 = 30

        # analog left
        left = (w/2 - w1/2) + w/2*self.a_joystick_left_x
        top = (w/2 - w1/2) + w/2*self.a_joystick_left_y
        self.app.evaluate_javascript("var div = document.getElementById('left-analog');div.style.left='"+str(left)+"px';div.style.top='"+str(top)+"px';")

        # analog right
        left = (w/2 - w1/2) + w/2*self.a_joystick_right_x
        top = (w/2 - w1/2) + w/2*self.a_joystick_right_y
        self.app.evaluate_javascript("var div = document.getElementById('right-analog');div.style.left='"+str(left)+"px';div.style.top='"+str(top)+"px';")

        # arrows
        self.color = "#cccccc"
        if self.up == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('up');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.down == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('down');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.left == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('left');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.right == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('right');div.style.backgroundColor='"+self.color+"'")

        # buttons
        self.color = "#cccccc"
        if self.select == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('select');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.start == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('start');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.ps == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('ps');div.style.backgroundColor='"+self.color+"'")


        self.color = "#cccccc"
        if self.triangle == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('triangle');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.circle == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('circle');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.cross == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('cross');div.style.backgroundColor='"+self.color+"'")
        self.color = "#cccccc"
        if self.square == 1:
            self.color = "#ff0000"
        self.app.evaluate_javascript("var div = document.getElementById('square');div.style.backgroundColor='"+self.color+"'")
