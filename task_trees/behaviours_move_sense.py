# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import rospy
from py_trees.common import Status
from std_msgs.msg import Bool
# robot control module
from task_trees.behaviours_base import ConditionalBehaviour
from task_trees.behaviours_move import DoMoveXYZ
from task_trees.tools.logging_tools import logger
# -----------------------------------------------------------------------------------------------------
# Advanced behaviours for illustrating the use of a ROS message topic to alert an imminent risk, such
# as collision, and to abort the move behaviour

# This class demonstrates the Behaviour Exception pattern
class DoMoveXYZGuardROS(DoMoveXYZ):
    """ This behaviour is based on DoMoveXYZ that moves the end-effector to a target_xyy position, with the additional feature of
        guarded by a ROS message topic that can trigger an emergency abort of the move command.
        This behaviour is coupled to ROS and saves the readings from a ROS topic to an instance variable. The ROS topic publishing
        True is an alert, that will abort the current command and the behaviour returns FAILURE.
    """   
    def __init__(self, name, condition_fn=True, policy=ConditionalBehaviour.Policies.SUCCESS_IF_FALSE_POLICY, arm_commander=None, 
                scene=None, target_xyz=None, reference_frame=None, cartesian=True, constraint_fn=None):
        """ the constructor, refers to the constructor of DoMoveXYZ
        """        
        super(DoMoveXYZGuardROS, self).__init__(name, condition_fn, policy, arm_commander, scene, target_xyz, reference_frame, cartesian, constraint_fn)
        # subscribe to the ROS topic that publishes the readings of a range finder
        self.alert_topic_sub = rospy.Subscriber('/sensor/alert', Bool, self._cb_alert_update)
        self.current_alert = False
        self.previous_alert = None # a variable for recording the previous state of too_close to reduce the verbosity of the info output
    
    # the callback for the ROS topic subscription
    def _cb_alert_update(self, msg:Bool):
        self.current_alert = msg.data
        
    # the concrete implementation of the logic when the General Commander is READY     
    def update_when_busy(self):
        # less verbose output of tracking the alert
        if self.previous_alert is None or self.previous_alert != self.current_alert:
            logger.info(f'DoMoveXYZGuardROS: alert changed: {self.current_alert}')
            self.previous_alert = self.current_alert
        # ask the General Commander to cancel the command and return FAILURE if too close is True 
        if self.current_alert:
            logger.error(f'DoMoveXYZGuardROS ({self.name}): alert received (collision is imminent)')
            self.arm_commander.abort_move()
            the_task = self.the_blackboard.task
            the_task.result = f'INTERRUPTED RAISED BY COLLISION DETECTION (collision alert)'
            return Status.FAILURE   
        return Status.RUNNING