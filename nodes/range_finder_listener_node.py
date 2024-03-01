#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# general modules
import rospy
from std_msgs.msg import Float32
import py_trees

class RangeFinderListener(): 
    """ This class implements a ROS node that listens to a ROS topic containing readings of a range finder
        and save the readings to the PyTrees blackboard
    """
    def __init__(self):
        # the ros topic to subscribe
        self.range_finder_sub_name = '/sensor/range_finder'
        # ros callback for shutdown
        rospy.on_shutdown(self.cb_shutdown)
        # subscribe to the topic
        self.range_finder_sub = rospy.Subscriber(self.range_finder_sub_name, Float32, self._cb_range_finder_update)
        # pytrees blackboard
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='range_finder', access=py_trees.common.Access.WRITE)        

    def cb_shutdown(self):
        rospy.loginfo('the ros node is being shutdown')

    def _cb_range_finder_update(self, msg:Float32):
        self.the_blackboard.range_finder = msg.data

# ----- execute the agent in a spin
def run_agent():
    try:
        rospy.loginfo('range finder listener node is running')
        robot_agent = RangeFinderListener()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

# ----- the main program
if __name__ == '__main__':
    rospy.init_node('range_finder_listener', anonymous=True)
    run_agent()