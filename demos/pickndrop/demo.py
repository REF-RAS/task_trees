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

import sys, signal, time
from enum import Enum
import py_trees
# robot control module
from arm_commander.commander_moveit import GeneralCommander
from task_trees_manager_pnd import PNDTaskTreesManager, ScanTask, PickUpObjectTask, DropObjectTask
from task_trees.states import TaskStates
from tools.logging_tools import logger
# -- Test cases specialized for the PickNDrop Demo

class DemoStates(Enum):
    INIT = 0  # generate the ball
    TO_SCAN = 1  # run the scan task
    SCAN = 2  # waiting for scan result
    PICK = 3  # run the pick up task
    DROP = 4  # run the drop task
    STOP = 5

class PNDDemoApplication():
    """ The application program for the PickNDrop Demo
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='seen_object', access=py_trees.common.Access.READ)  
        
        self.arm_commander = GeneralCommander('panda_arm')
        self.the_task_manager = PNDTaskTreesManager(self.arm_commander)

        self.to_stop = False
        self._run_demo()
        self.the_task_manager.spin()

    def stop(self, *args, **kwargs):
        logger.info('stop signal received')
        self.the_task_manager.shutdown()
        sys.exit(0)

    def _run_scan(self):
        task_manager = self.the_task_manager
        logger.info(f'=== Test 1')       
        
        logger.info(f'=== Submit a ScanTask')
        the_task = ScanTask()
        task_manager.submit_task(the_task)
        the_task.wait_for_completion()     
        logger.info(f'=== Test Finished')

    def _run_demo(self):
        task_manager = self.the_task_manager
        state = DemoStates.INIT
        the_task = None
        logger.info(f'=== PickNDrop Demo Started') 
        scene_setup = False
        while not self.to_stop:
            time.sleep(0.1)
            if state == DemoStates.INIT:
                logger.info(f'=== PickNDrop INIT: Initialization of Pose and Submit Scan Task')
                task_manager.submit_task(the_task := ScanTask()) 
                state = DemoStates.SCAN
                logger.info(f'=== PickNDrop INIT: Waiting for the Scan Task to WORKING')
                the_task.wait_for_working()
                logger.info(f'=== PickNDrop INIT: The Scan Task is WORKING, setup the scene if needed and generate a new target')
                if not scene_setup:
                    task_manager.setup_objects()
                    scene_setup = True
                self.the_task_manager.generate_a_ball()
            elif state == DemoStates.SCAN:
                if the_task.state not in [TaskStates.WORKING]:
                    logger.info(f'=== PickNDrop SCAN: Scan Failed and State changed to STOP')
                    state = DemoStates.STOP    
                self.the_task_manager.simulate_camera()
                if self.the_blackboard.exists('seen_object'):
                    xyz = self.the_blackboard.seen_object
                    logger.info(f'=== PickNDrop SCAN: An object Found at {xyz} and so Cancel the Scantask and Wait')
                    the_task.cancel(wait=True)
                    logger.info(f'=== PickNDrop SCAN: Submit PickUpObjectTask')                    
                    task_manager.submit_task(the_task := PickUpObjectTask(xyz_world=xyz))
                    state = DemoStates.PICK
            elif state == DemoStates.PICK:
                the_task.wait_for_completion()
                if the_task.get_state() not in [TaskStates.SUCCEEDED]:
                    logger.info(f'=== PickNDrop PICK: Pick Failed and State changed to STOP')
                    state = DemoStates.STOP
                else:
                    logger.info(f'=== PickNDrop PICK: Pick Successful and Submit Drop Task')
                    task_manager.submit_task(the_task := DropObjectTask()) 
                    state = DemoStates.DROP
            elif state == DemoStates.DROP:
                the_task.wait_for_completion() 
                if the_task.get_state() in [TaskStates.SUCCEEDED]:
                    logger.info(f'=== PickNDrop DROP: Drop Successful')
                    state = DemoStates.INIT
                else:
                    logger.info(f'=== PickNDrop PICK: Drop Failed and State changed to STOP')
                    state = DemoStates.STOP
            elif state == DemoStates.STOP:
                logger.info(f'=== PickNDrop STOP')    
                return            

if __name__=='__main__':
    # rospy.init_node('run_pnd_task_manager_demo', anonymous=False)
    try:
        PNDDemoApplication()
        logger.info('pnd_task_manager_demo is running')
    except Exception as e:
        logger.exception(e)


