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
import rospy
from std_msgs.msg import Int8
from task_trees.tools.logging_tools import logger

# Require the python Tkinter module
# sudo apt install python3-tk

class TaskDemoGUI():
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        self.listener = None
        self.to_stop = False
        self.estop_state = False  # the state of the estop button
        # create the GUI
        self.win = GraphWin('Task Submit GUI', 200, 100)
        self.draw_button(50, 50, '1')
        self.draw_button(150, 50, '2')        
        self.to_stop = False
        self.task_pub = rospy.Publisher('/taskdemo/do', Int8, queue_size=10)
        self.gui_loop()
    
    # Draws a button with a text label in the middle
    def draw_button(self, x, y, text):
        self.button = Circle(Point(x, y), 30)
        self.button.setFill('light blue')
        self.button.draw(self.win)
        self.text_msg = Text(Point(x, y), text)
        self.text_msg.setStyle('bold')
        self.text_msg.draw(self.win)        
        
    # The callback for receiving SIGINT interrupt signal
    def stop(self, *args, **kwargs):
        logger.info('stop signal received')
        self.to_stop = True
        self.win.close()
        sys.exit(0)
    
    # The main loop that listens for mouse click on the GUI
    def gui_loop(self):
        while True:
            if self.to_stop:
                break
            time.sleep(0.1)
            if self.win.closed:
                sys.exit(0)
            # check for mouse click on the window without blocking
            clickPoint = self.win.checkMouse()
            if clickPoint is not None:
                if clickPoint.x < 100:
                    self.task_pub.publish(Int8(1))
                else:
                    self.task_pub.publish(Int8(2))
                    
# -----------------------------------
# The main program

if __name__== '__main__':
    rospy.init_node('task_submit_gui', anonymous=False)
    try:
        the_estop = TaskDemoGUI()
        logger.info('task_submit_gui is running')
    except Exception as e:
        logger.exception(e)