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
from py_trees.behaviour import Behaviour
from py_trees.common import Status

# robot control module
from task_trees.behaviours_base import ConditionalBehaviour, ConditionalCommanderBehaviour
import arm_commander.moveit_tools as moveit_tools
from scan_model import ScanRegionModel

# ----------------------------------------------------------------------
# Behaviour Classes for the Pick-n-Drop robot arm manipulation application

class DoScanProgram(ConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector over a planar region according toa program
        The orientation of the end-effector is fixed.
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, scan_program:ScanRegionModel=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param scan_program: the scan program
        :type scan_program: ScanRegionModel
        """
        super(DoScanProgram, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, arm_commander=arm_commander)
        if scan_program is None:
            rospy.logerr(f'{__class__.__name__} ({self.name}): parameter (scan_program) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')         
        self.scan_program:ScanRegionModel = scan_program
    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        xyz = self.scan_program.get_next_xyz()
        if xyz is None:
            self.scan_program.reset()
            xyz = self.scan_program.get_next_xyz()
            
        # a heck to test the use of orientation constraint
        self.arm_commander.add_path_constraints(moveit_tools.create_path_orientation_constraint(
            self.arm_commander.END_EFFECTOR_LINK, self.arm_commander.pose_in_frame('the_table'), 0.05, 0.05, 6.28))
        # send the command to the General Commander in an asynchronous manner
        self.arm_commander.move_to_position(x=xyz[0], y=xyz[1], z=xyz[2], cartesian=True, reference_frame='the_table', wait=False)
        rospy.loginfo(f'DoScanProgram ({self.name}): started move to pose: {xyz} in reference frame "the_table"')   
        return Status.RUNNING
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.scan_program.done_current()
        self.arm_commander.clear_path_constraints()

    