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
from std_msgs.msg import Bool

# Require the python Tkinter module
# sudo apt install python3-tk

class EStop():
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        self.to_stop = False
        self.estop_state = False  # the state of the estop button
        # Create the GUI window and draw the e-stop button
        self.win = GraphWin('E-Stop GUI', 100, 100)
        self.button = Circle(Point(50, 60), 30)
        self.button.setFill('dark red')
        self.button.draw(self.win)
        self.text_msg = Text(Point(50, 15), 'EStop OFF')
        self.text_msg.setStyle('bold')
        self.text_msg.draw(self.win)
        # set up a publisher for the estop status
        self.estop_pub = rospy.Publisher('/estop', Bool, queue_size=10)
        # run the UI loop
        self.gui_loop()
        
    # The callback for receiving SIGINT interrupt signal        
    def stop(self, *args, **kwargs):
        rospy.loginfo('stop signal received')
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
                self.estop_state = not self.estop_state
                if self.estop_state:
                    self.button.setFill('red')
                    self.text_msg.setText('EStop ON')
                else:
                    self.button.setFill('dark red')
                    self.text_msg.setText('EStop OFF')
            self.estop_pub.publish(Bool(self.estop_state))

# -----------------------------------
# The main program
if __name__== '__main__':
    rospy.init_node('estop_simulator', anonymous=False)
    try:
        the_estop = EStop()
        rospy.loginfo('estop simulator is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
