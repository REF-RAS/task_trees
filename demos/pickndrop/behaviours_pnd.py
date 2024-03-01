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
from work_model import ScanRegionModel

# ----------------------------------------------------------------------
# Behaviour Classes for the Pick-n-Drop robot arm manipulation application

class DoScanProgram(ConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector over a planar region according toa program
        The orientation of the end-effector is fixed.
    """
    def __init__(self, name, condition_fn=True, policy=ConditionalBehaviour.Policies.SUCCESS_IF_FALSE_POLICY, arm_commander=None, scan_program:ScanRegionModel=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param scan_program: the scan program
        :type scan_program: ScanRegionModel
        """
        super(DoScanProgram, self).__init__(name, condition_fn, policy, arm_commander)
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
        rospy.loginfo(f'DoScanProgram ({self.name}): started move to pose: {xyz}')   
        return Status.RUNNING
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.scan_program.done_current()
        self.arm_commander.clear_path_constraints()
    

class SimGrabObject(ConditionalCommanderBehaviour):
    """ This behaviour simulates the grab of an object
    """
    def __init__(self, name, condition_fn=True, policy=ConditionalBehaviour.Policies.SUCCESS_IF_FALSE_POLICY, arm_commander=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        """
        super(SimGrabObject, self).__init__(name, condition_fn, policy, arm_commander)

    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):       
        # send the command to the General Commander in an asynchronous manner
        rospy.loginfo(f'SimGrabObject: grab the object')         

        self.arm_commander.attach_object_to_end_effector('the_ball')
        return Status.SUCCESS
    
        
class SimDropObject(ConditionalCommanderBehaviour):
    """ This behaviour simulates dropping the object
    """
    def __init__(self, name, condition_fn=True, policy=ConditionalBehaviour.Policies.SUCCESS_IF_FALSE_POLICY, arm_commander=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        """
        super(SimDropObject, self).__init__(name, condition_fn, policy, arm_commander)

    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):       
        # send the command to the General Commander in an asynchronous manner
        rospy.loginfo(f'SimGrabObject: drop the object')         
        self.arm_commander.detach_object_from_end_effector('the_ball')
        self.arm_commander.remove_object('the_ball')
        return Status.SUCCESS
    