#
# Author: Alberto Petrucci (petrucci.alberto@gmail.com) 2017
#
__author__ = "Alberto Petrucci"
__copyright__ = "Copyright 2017, Alberto Petrucci"
__credits__ = ["Alberto Petrucci"]
__license__ = "MIT"
__version__ = "1.0.0"
__maintainer__ = "Alberto Petrucci"
__email__ = "petrucci.alberto@.com"
__status__ = "Production"

import htmlPy
import sys, os
from PyQt4.QtGui import *
from subprocess import *

class BackEnd(htmlPy.Object):
    def __init__(self, app):
        super(BackEnd, self).__init__()
        self.app = app
        self.base_path = os.path.dirname(sys.argv[0])

    def get_file_contents(self, filename):
        file = open(filename, 'r')
        content = file.read()
        file.close()
        content = content.replace("{% include 'header.html' %}", "")
        content = content.replace("{% include 'footer.html' %}", "")
        return content

    @htmlPy.Slot(str)
    def log(self, string):
        """ Prints the string to python console.
        The method is binded to GUI javascript. Thus, the string comes from GUI
        Arguments:
        string (str): The string to be printed.
        """
        print(string)

    @htmlPy.Slot(str, result=str)
    def openJson(self, file):
        if file is not None:
            fileName = os.path.abspath(self.base_path)+'/IndoorPositionEstimator/'+file+'.json'
        else:
            # The QWidget widget is the base class of all user interface objects in PyQt4.
            w = QWidget()

            # Set window size.
            w.resize(320, 240)

            # Set window title
            w.setWindowTitle("Open JSON")

            fileName = QFileDialog.getOpenFileName(w, "Open JSON", '', "JSON Files (*.json)")
        file = open(fileName, 'r')
        content = file.read()
        file.close()
        return content

    @htmlPy.Slot(str, str)
    def saveJson(self, jsonStr="[]", file=None):
        if file is not None:
            fileName = os.path.abspath(self.base_path)+'/IndoorPositionEstimator/'+file+'.json'
        else:
            # The QWidget widget is the base class of all user interface objects in PyQt4.
            w = QWidget()

            # Set window size.
            w.resize(320, 240)

            # Set window title
            w.setWindowTitle("Export Sequence")

            fileName = QFileDialog.getSaveFileName(w, "Export Sequence", 'sequence', "JSON Files (*.json)")
        file = open(fileName, 'w')
        file.write(jsonStr)
        file.close()

    @htmlPy.Slot(result=str)
    def home_page(self):
        #self.app.template = ("index.html", {"active_menu": "1"})
        filename = os.path.abspath(self.base_path)+'/templates/index.html'
        return self.get_file_contents(filename)

    @htmlPy.Slot(result=str)
    def joystick_page(self):
        #self.app.template = ("joystick.html", {"active_menu": "2"})
        filename = os.path.abspath(self.base_path)+'/templates/joystick.html'
        return self.get_file_contents(filename)

    @htmlPy.Slot(result=str)
    def sequence_page(self):
        #self.app.template = ("sequence.html", {"active_menu": "3"})
        filename = os.path.abspath(self.base_path)+'/templates/sequence.html'
        return self.get_file_contents(filename)

    @htmlPy.Slot()
    def start_sequence(self):
        pathname = os.path.dirname(sys.argv[0])
        path = os.path.abspath(pathname) + '/IndoorPositionEstimator'
        fileName = path + '/panda3dCube.py'
        call("cd '" + path + "' && python '" + fileName + "'", shell=True)
