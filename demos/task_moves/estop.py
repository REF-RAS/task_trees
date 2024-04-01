#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import time, signal
from graphics import *

# Require the python Tkinter module
# sudo apt install python3-tk

class EStop():
    def __init__(self, listener=None):
        self.listener = listener  # a function to call when the state changes 
        self.estop_state = False  # the state of the estop button
        # Create the GUI window and draw the e-stop button
        self.win = GraphWin('E-Stop GUI', 100, 100)
        self.button = Circle(Point(50, 60), 30)
        self.button.setFill('dark red')
        self.button.draw(self.win)
        self.text_msg = Text(Point(50, 15), 'EStop OFF')
        self.text_msg.setStyle('bold')
        self.text_msg.draw(self.win)
    
    def get_estop_state(self):
        return self.estop_state
        
    # The main loop that listens for mouse click on the GUI    
    def gui_loop(self):
        while True:
            time.sleep(0.1)
            if self.win.closed:
                sys.exit(0)
            # check for mouse click on the window without blocking
            clickPoint = self.win.checkMouse()
            if clickPoint is not None:
                self.estop_state = not self.estop_state
                if self.estop_state:
                    self.button.setFill('red')
                    self.text_msg.setText('EStop ON')
                else:
                    self.button.setFill('dark red')
                    self.text_msg.setText('EStop OFF')
            if self.listener is not None:
                self.listener(self.estop_state)


