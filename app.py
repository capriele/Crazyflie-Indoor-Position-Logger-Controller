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
import os
from PySide import QtGui
from back_end import BackEnd
from math_parser import MathParser
from joystick import Joystick

# Initial confiurations
BASE_DIR = os.path.abspath(os.path.dirname(__file__))

# GUI initializations
#app = htmlPy.AppGUI(title=u"CrazyFlie Indoor Position Logger + Controller", maximized=False, plugins=True, developer_mode=True)
app = htmlPy.AppGUI(title=u"CrazyFlie Indoor Position Logger + Controller", maximized=False, plugins=True)

# GUI configurations
app.static_path = os.path.join(BASE_DIR, "static/")
app.template_path = os.path.join(BASE_DIR, "templates/")
app.web_app.setMinimumWidth(1024)
app.web_app.setMinimumHeight(768)
app.window.setWindowIcon(QtGui.QIcon(BASE_DIR + "/static/img/icon.png"))

app.bind(BackEnd(app))
app.bind(MathParser(app))
app.bind(Joystick(app))

# Disabilito right-click del mouse
app.right_click_setting(False)

# Disabilito selezione testo
app.text_selection_setting(False)

app.template = ("index.html", {"active_menu": "1"})

# Instructions for running application
if __name__ == "__main__":
    # The driver file will have to be imported everywhere in back-end.
    # So, always keep app.start() in if __name__ == "__main__" conditional
    app.start()
