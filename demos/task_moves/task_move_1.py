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

import sys, signal, random
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector
# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
from task_trees.behaviours_move import DoMoveJointPose, DoMoveDisplaceXYZ
from task_trees.task_trees_manager import TaskTreesManager, BasicTask

class MoveRectTask(BasicTask):
    def __init__(self):
        """ the constructor for a task moving in a rectangle pattern
        """
        super(MoveRectTask, self).__init__()

# ---------------------------------------
# The TaskManager specialized for this application
class SimpleTaskMoveManager(TaskTreesManager):
    """ This is a subclass of TaskManager, for illustration of using the framework for developing behaviour trees
        The behaviour tree contains one behaviour, which moves the end effector in a rectangular circuit, each side divided into 5 steps of displacement
        On the last side of the rectangle, the step size is random between 0.01 and 0.09 meters. 
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(SimpleTaskMoveManager, self).__init__(arm_commander)
        # build and install the behavior tree
        self._set_initialize_branch(self.create_init_branch())
        self._add_task_branch(self.create_move_rect_branch(), MoveRectTask)

        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
        
    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    # returns a behaviour tree branch that move the end_effector in a rectanglar path
    def create_move_rect_branch(self) -> Composite:
        """ Returns a behaviour tree branch that move the end_effector in a rectangular path
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = Sequence( 
                'move_branch',
                memory=True,
                children=[
                    DoMoveDisplaceXYZ('move_dy', True, arm_commander=self.arm_commander, dxyz=[0.0, 0.3, 0]), 
                    DoMoveDisplaceXYZ('move_dz', True, arm_commander=self.arm_commander, dxyz=[0, 0, 0.3]), 
                    DoMoveDisplaceXYZ('move_ndy', True, arm_commander=self.arm_commander, dxyz=[0, -0.3, 0]), 
                    DoMoveDisplaceXYZ('move_random_ndz', True, arm_commander=self.arm_commander, 
                                      dxyz=lambda: [0.0, 0.0, random.uniform(-0.25, -0.35)]),               
                    ],
        )
        return move_branch
    
    # returns a behaviour tree branch that performs initialzation of the robot by moving to a prescribed pose
    def create_init_branch(self) -> Composite:
        init_branch = Sequence('init_branch', memory=True,
                children=[DoMoveJointPose('reset_pose', True, arm_commander=self.arm_commander, 
                                    target_joint_pose=[0.00, -1.243, 0.00, -2.949, 0.00, 1.704, 0.785],), ],)
        return init_branch              
   
   
class TaskDemoApplication():
    """ The application program for the simple Task Demo
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        
        self.arm_commander = GeneralCommander('panda_arm')
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        self.the_task_manager = SimpleTaskMoveManager(self.arm_commander)
        # self.the_task_manager.display_tree()
        self._run_demo()
        
    def stop(self, *args, **kwargs):
        logger.info('stop signal received')
        self.the_task_manager.shutdown()
        sys.exit(0)
        
    def _run_demo(self):
        task_manager = self.the_task_manager
        the_task = None
        logger.info(f'=== Task Demo Started') 
        for i in range(10):
            logger.info(f'=== Submit a MoveRectTask #{i + 1}')
            task_manager.submit_task(the_task:=MoveRectTask())
            the_task.wait_for_completion()    

if __name__=='__main__':
    # rospy.init_node('simple_task_demo', anonymous=False)
    try:
        TaskDemoApplication()
        logger.info('simple_task_demo is running')
    except Exception as e:
        logger.exception(e)

