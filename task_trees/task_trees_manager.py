# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import threading, operator, signal, sys, time, logging
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.decorators import EternalGuard
from py_trees.trees import BehaviourTree

from task_trees.behaviours_base import *
from task_trees.states import TaskStates, COMPLETION_STATES
from task_trees.tools import logger

# -- The abstract class for implementing project specific Task for the TaskTreesManager
class BasicTask():
    """ It defines the essential parameters for the lifecycle management by the TaskTreesManager 
    """
    def __init__(self, goal_as_logical_pose=None):
        
        self.task_lock = threading.Lock()
        # task parameters
        self.name = self.__class__                          # the class name for identification of the task
        self.goal_as_logical_pose = goal_as_logical_pose    # the main goal of the task - assume to be a logical pose
        self.xyz = None                                     # the mapped physical pose in xyz format
        self.rpy = None                                     # the mapped physical pose in euler angles in rpy order 
        # task state
        self.state = TaskStates.STANDBY                     # the init state
        self.result = ''                                    # the task result, which will be assigned by the TaskTreesManager at completion
        self.commander_feedback:str = None
        # the cancel function of the task manager
        self._cancel_fn = None                              # internal use
        # the time submitted to the task manager
        self._submit_time = None                            # internal use
    
    # function for updating the state in a thread-safe manner
    def update_state(self, new_state:TaskStates):
        """ update the state (thread-safe)

        :param new_state: the new state
        :type new_state: TaskStates
        """
        self.task_lock.acquire()
        try:
            self.state = new_state
        finally:
            self.task_lock.release()
            
    def get_state(self) -> TaskStates:
        """ return the state of the task
        :return: the current state of the task
        :rtype: TaskStates
        """
        return self.state
    
    # internal function used by the abstract TaskTreesManager
    def _set_submitted(self, cancel_fn):
        """ The task has been submitted to the TaskTreesManager, and consequently the state of this task is updated and
            the callback function for task cancellation is stored (to de-couple the )
        :param cancel_fn: the cancel function provided by the Task Manager
        :type cancel_fn: func
        """
        self.update_state(TaskStates.SUBMITTED)
        self._cancel_fn = cancel_fn
        self._submit_time = time.time()
        
    def cancel(self, wait=True) -> bool:
        """ cancel the task

        :return: True if the task has been submitted and the cancellation is sucessful.
        :rtype: bool
        """
        if self._cancel_fn is not None:
            result = self._cancel_fn(self)
            if wait:
                self.wait_for_completion()
            return result
        logger.error('This task has not been submitted -> check program logic')
        return False

    def get_goal_as_logical_pose(self):
        """ returns the main goal of the task, which is an logical pose

        :return: the goal as a logical pose
        :rtype: arbitrary, dependent on the subclass definition
        """
        return self.goal_as_logical_pose
    
    def get_xyz(self) -> list:
        """ return the position of the target pose in xyz 
        :return: the position of the target pose in xyz 
        :rtype: list
        """
        return self.xyz
    
    def get_rpy(self) -> list:
        """ return the orientation part of the target pose 
        :return: the orientation part of the target pose 
        :rtype: list
        """
        return self.rpy
    
    def wait_for_completion(self, until_states:list=None):
        """ a blocking function that returns when the task has entered into one of the :param:until_states

        :param until_states: _description_, defaults to None
        :type until_states: list, optional
        :return: _description_
        :rtype: _type_
        """
        if until_states is None:
            until_states = COMPLETION_STATES
        previous_state = None
        while True:
            if self.state in until_states:
                return self.state
            if previous_state is None or previous_state != self.state:
                logger.info(f'{type(self).__name__} (wait_for_completion): the thread is waiting (task_state: {self.state}) ...')
                previous_state = self.state
            time.sleep(0.5)
    
    def wait_for_working(self):
        """ a blocking function that returns when the task has entered into WORKING state or GUARDED_ABORTED state
        """
        previous_state = None
        while True:
            if self.state in [TaskStates.WORKING, TaskStates.GUARD_ABORTED]:
                return
            if previous_state is None or previous_state != self.state:
                logger.info(f'wait_for_working task_state: {self.state} ...')
                previous_state = self.state
            time.sleep(0.5)             
                        
    def get_time_since_submit(self) -> float:
        """ return the time lapse since submission
        :return: the time lapse since submission, or -1 if the task has not been submitted
        :rtype: float
        """
        if self._submit_time is None:
            return -1
        return time.time() - self._submit_time
    
    def get_commander_feedback(self) -> str:
        """ return the final feedback from the commander for error checking
        :return: the final message
        :rtype: str
        """        
        return self.commander_feedback

# -----------------------------------------------------------------------------------------------------------
# The generic Basic Task Manager for managing the life cycle of the actions and behaviours for various tasks  
class BasicTaskTreesManager:
    """ subclass the BasicTaskTreesManager to develop a project specific task manager and its behaviour trees.
    """
    def __init__(self):
        """ the constructor
        """
        signal.signal(signal.SIGINT, self.stop)
        self.to_shutdown = False
        # setup the blackboard
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='task', access=py_trees.common.Access.WRITE)
        # the behaviour tree to be built
        self.bt = None
        
    # internal function for a subclass to install the sepcific behaviour trees and start the tick-tocking 
    def _install_bt_and_spin(self, bt:py_trees.trees.BehaviourTree, spin_period_ms:int=10, startup_wait_sec=3.0):
        """ The final function to call, which installs the behaviour trees and starts the tick-tocking at the given frequency.
        Normally, this function does not return.
        :param bt: the behaviour tree
        :type bt: py_trees.trees.BehaviourTree
        :param spin_period_ms: the period of the tick-tocking, defaults to 10 ms
        :type spin_period_ms: int, optional
        """
        if bt is None:
            logger.error(f'{__class__.__name__}: parameter (bt) is None -> fix the missing value at function call to _install_bt_and_spin')
            raise AssertionError(f'A parameter should not be None nor missing') 
        self.bt = bt
        time.sleep(startup_wait_sec)
        # spin the tree
        self.the_thread = threading.Thread(target=lambda: 
            self.bt.tick_tock(period_ms=spin_period_ms, 
                              pre_tick_handler=self._pre_tick_handler,
                              post_tick_handler=self._post_tick_handler), daemon=True)
        self.the_thread.start()  
    
    # internal function: called by the task when it receives a cancel call
    def _cancel_task(self, the_task:BasicTask) -> bool:
        if self.the_blackboard.exists('task'):
            logger.warning(f'BasicTaskTreesManager (_cancel_task): {type(the_task).__name__} is CANCELLED (task state:{self.the_blackboard.task.state})')
            if self.the_blackboard.task is not None and self.the_blackboard.task == the_task:
                if self.the_blackboard.task.state in [TaskStates.SUBMITTED, TaskStates.WORKING]:
                    self.the_blackboard.task.update_state(TaskStates.CANCELLED)
                    return True
        return False
    
    # internal function: the pre-handler for tick-tock
    def _pre_tick_handler(self, tree):
        pass
    
    # internal function: the post-handler for tick-tock
    def _post_tick_handler(self, tree):
        pass    
    
    # ------------------------------------------------------------
    # Task Trees functions for servicing the application

    # function for handling the SIGINT signal
    def stop(self, *args, **kwargs):
        logger.warning(f'{__class__.__name__}: SIGNINT signal received -> interrupt the tick-tock of the behaviour tree and shutdown the task manager')
        self.shutdown()
        sys.exit(0)

    # function for preventing the main thread from terminating the application
    def spin(self):
        while not self.to_shutdown:
            time.sleep(1.0)

    # shutdown the task manager
    def shutdown(self):
        """ Shutdown the task manager immediately
        """
        # terminate the behaviour tree tick-tock
        if self.bt is not None:
            self.bt.interrupt()
            self.bt.shutdown()
        else:
            logger.warning(f'{__class__.__name__}: attempted to shutdown a task manager with no behaviour tree installed')
        # terminate the spin
        self.to_shutdown = True
        
    # returns the submitted task   
    def get_submitted_task(self) -> BasicTask:
        """ Return the submitted task
        :return: The submitted task, or None
        :rtype: BasicTask
        """
        if self.the_blackboard.exists('task'):
            return self.the_blackboard.task
        return None
    
    # allows a client application to submit a task to this task manager
    def submit_task(self, the_task:BasicTask) -> bool:
        """ The window for task submission.
        :param the_task: The task to be executed
        :type the_task: a subclass of robotarchi.BasicTask
        :return: True if the task manager is in the states that can accept the task submission 
        :rtype: bool
        """
        
        if self.the_blackboard.exists('task'):
            if self.the_blackboard.task.state in [TaskStates.SUBMITTED, TaskStates.WORKING]:
                logger.error(f'TaskTreesManager: submit a new task when a current task is SUBMITTED or WORKING')
                return False
        the_task._set_submitted(self._cancel_task)
        self.the_blackboard.set('task', the_task, overwrite=True) 
        return True
        
    # a utility function for rendering a the behaviour tree of this task manager as a dot file
    def display_tree(self):
        """ renders the behaviour tree of this task manager as a dot file
        """
        if self.bt is None:
            logger.error(f'{__class__.__name__}: attempted to display the bt of a task manager without one installed -> fix the subclass by assigning a behaviour tree to self.bt')
            raise AssertionError(f'A subclass of {__class__.__name__} has not assigned a behaviour tree to self.bt')         
        py_trees.display.render_dot_tree(self.bt.root)


# -----------------------------------------------------------------------------------------------------------
# The generic Task Manager to support the framework of behaviour tree development 
# it is for managing the life cycle of the actions and behaviours for various tasks  
class TaskTreesManager(BasicTaskTreesManager):
    """ subclass the TaskTreesManager to develop application specific behaviour trees that are structures to support
        the plug-in of tasks, timeout, and on-shot initialization branches 
    """
    def __init__(self, arm_commander):
        """ the constructor
        """
        super(TaskTreesManager, self).__init__()
        if arm_commander is None:
            logger.error(f'{__class__.__name__}): parameter (arm_commander) is None -> fix the missing value at TaskTreesManager construction')
            raise AssertionError(f'A parameter should not be None nor missing') 
        self.arm_commander:GeneralCommander = arm_commander
        self.bt = self._build_tree_skeleton()
        self.num_initialize_branch = 0
        self.num_priority_branch = 0

    # shutdown the task manager
    def shutdown(self):
        self.arm_commander.abort_move(wait=True)
        super().shutdown()

    def _build_tree_skeleton(self):
        self.do_task_selector = py_trees.composites.Selector('do_tasks_selector_branch', memory=True, children=[
                Print(f'One of the behaviours in the do_work_branch FAILED', print_tree=False),
                HandleTaskFailure('handle_task_failure', self.arm_commander),
            ],)
        self.cancellable_do_tasks_branch = py_trees.composites.Sequence (
            'cancellable_do_tasks_branch', memory=False, children = [
                HandleTaskCancelled('handle_cancel_task', self.arm_commander),
                self.do_task_selector,
            ]
        )
        do_tasks_trunk = py_trees.composites.Sequence(
            'do_work_trunk',
            memory=True,
            children=[
                py_trees.behaviours.CheckBlackboardVariableExists(name='check_task_exists', variable_name='task'),
                py_trees.behaviours.CheckBlackboardVariableValue(name='check_task_state', check=py_trees.common.ComparisonExpression(
                            variable='task.state', value=TaskStates.SUBMITTED, operator=operator.eq)),
                self.cancellable_do_tasks_branch,
            ],
        )
        self.root_sequence = py_trees.composites.Sequence('root_sequence', memory=True, children=[
            do_tasks_trunk,
            ])
        self.the_root = BehaviourTree(self.root_sequence) 
        return self.the_root
        
    def _set_initialize_branch(self, branch:Composite):
        if self.num_initialize_branch > 0:
            logger.error(f'{__class__.__name__} (_set_initialize_branch): cannot set initialization branch more than once-> fix the subclass and avoid calling the function more then once')
            raise AssertionError(f'A parameter should not be None nor missing')  
        self.initialize_branch = py_trees.decorators.OneShot('init_oneshot_branch', policy = py_trees.common.OneShotPolicy.ON_COMPLETION, child=branch)
        self.root_sequence.insert_child(self.initialize_branch, 0)
        self.num_initialize_branch = 1
        
    def _add_priority_branch(self, branch:Composite):
        index = self.num_initialize_branch + self.num_priority_branch
        self.root_sequence.insert_child(branch, index)
        self.num_priority_branch += 1
    
    def _add_task_branch(self, branch:Composite, task_class:type=None):
        if task_class is None:
            logger.warning(f'TaskTreesManager (_add_task_branch): task_class is None -> task decorations are assumed to be included in the parameter branch or provide the task_class at the function call')
            task_branch = branch
        else:
            # - add a composite to the task branch
            task_branch = py_trees.composites.Sequence(
                f'task_branch_for_{task_class.__name__.lower()}',
                memory=True,
                children=[     
                    # add the task class filter
                    py_trees.behaviours.CheckBlackboardVariableValue(
                                name='check_scan_task', check=py_trees.common.ComparisonExpression(variable='task.name', value=task_class, operator=operator.eq)),              
                    py_trees.behaviours.SetBlackboardVariable(
                                name='set_state_to_working', variable_name='task.state', variable_value=TaskStates.WORKING, overwrite=True),
                    branch,
                    # add the task success reporter
                    py_trees.behaviours.SetBlackboardVariable(name='set_task_success', variable_name='task.state', 
                                variable_value=TaskStates.SUCCEEDED, overwrite=True),
            ])
        # now add the new branch to the skeleton
        num_children = len(self.do_task_selector.children)
        self.do_task_selector.insert_child(task_branch, num_children - 2)  

    def display_tree(self, branch=None, target_directory=None):
        branch = self.bt.root if branch is None else branch
        if branch is None:
            return
        logger.info(f'display tree {branch} {target_directory}')
        py_trees.display.render_dot_tree(branch, target_directory=target_directory)


# -----------------------------------------------------------------------------------------------------------
# The generic Guarded Task Manager to support the framework of behaviour tree development 
# it is for managing the life cycle of the actions and behaviours for various tasks  
# the difference from TaskTreesManager is the option of setting conditions to invalidate the behaviour at every tick-tock
class GuardedTaskTreesManager(TaskTreesManager):
    """ subclass the TaskTreesManager to develop application specific behaviour trees that are structures to support
        the plug-in of tasks, timeout, and on-shot initialization branches 
    """
    def __init__(self, arm_commander:GeneralCommander, global_guard_condition_fn=None, task_guard_condition_fn=None, guard_reset=True):
        """The constructor for this class

        :param arm_commander: The commander to be used for the behaviours in the task tree
        :type arm_commander: GeneralCommander
        :param global_guard_condition_fn: the condition function that trigger the global guard for the task trees, defaults to None
        :type global_guard_condition_fn: a function returning a bool type, True if no alert, optional
        :param task_guard_condition_fn: the condition function that trigger the task guard for the task trees, defaults to None
        :type task_guard_condition_fn: a function returning a bool type, True if no alert, optional
        :param guard_reset: whether an activated guard needs reset to go back to unactivated state, defaults to True
        :type guard_reset: bool, optional
        """
        self.global_guard_activated = False
        self.task_guard_activated = False
        self.custom_global_guard_condition_fn = self.custom_task_guard_condition_fn = None
        self.set_global_guard_condition_fn(global_guard_condition_fn)
        self.set_task_guard_condition_fn(task_guard_condition_fn)        
        self.guard_reset = guard_reset
        super(GuardedTaskTreesManager, self).__init__(arm_commander)
        
    # internal function to handle the activation of the EternalGuard
    def _post_tick_handler(self, tree):  
        if self.global_guard.status == Status.FAILURE and self.global_guard.condition() == False:
            self.global_guard_activated = True
        elif self.guard_reset == False:
                self.global_guard_activated = not self.global_guard.condition() 
            
        if self.task_guard.status == Status.FAILURE and self.task_guard.condition() == False:
            self.task_guard_activated = True 
        elif self.guard_reset == False:
                self.task_guard_activated = not self.task_guard.condition() 

        if (self.global_guard_activated or self.task_guard_activated) and self.the_blackboard.exists('task'):
            the_task:BasicTask = self.the_blackboard.get('task')
            if the_task is not None:
                the_task.state = TaskStates.GUARD_ABORTED
                self.the_blackboard.unset('task')  
        
    # internal function: the condition function for the eternal guards
    
    # return True if no alert should be made 
    def _get_global_guard_condition(self):
        #if self.custom_global_guard_condition_fn is not None:
        #    logger.info(f'custom_global_guard_condition_fn: {self.custom_global_guard_condition_fn()}')
        return not self.global_guard_activated and (self.custom_global_guard_condition_fn is None or self.custom_global_guard_condition_fn())
    
    # return True if no alert should be made    
    def _get_task_guard_condition(self):
        return not self.task_guard_activated and (self.custom_task_guard_condition_fn is None or self.custom_task_guard_condition_fn())

    # setting the guard conditions
    def set_global_guard_condition_fn(self, global_guard_condition_fn):
        if global_guard_condition_fn is not None and not hasattr(global_guard_condition_fn, '__call__'):
            raise AssertionError(f'GuardedTaskTreesManager: The parameter root_guard_condition is not a function')
        self.custom_global_guard_condition_fn = global_guard_condition_fn

    def set_task_guard_condition_fn(self, task_guard_condition_fn):
        if task_guard_condition_fn is not None and not hasattr(task_guard_condition_fn, '__call__'):
            raise AssertionError(f'GuardedTaskTreesManager: The parameter task_guard_condition is not a function')
        self.custom_task_guard_condition_fn = task_guard_condition_fn       
    
    # override the submit_task method
    def submit_task(self, the_task:BasicTask) -> bool:
        if self.global_guard_activated or self.task_guard_activated:
            logger.error(f'GuardedTaskTreesManager: submit a new task when the Guard is still activated -> call reset_guard before submitting tasks')            
            return None
        return BasicTaskTreesManager.submit_task(self, the_task)
    
    # reset the guard activation
    def reset_guard(self):
        if (self.custom_global_guard_condition_fn is not None and self.custom_global_guard_condition_fn() == False) or \
            (self.custom_task_guard_condition_fn is not None and self.custom_task_guard_condition_fn() == False):
            logger.warning(f'{__class__.__name__} (reset_guard): Unable to reset guard activation status because the guard condition is still False')
            return
        self.global_guard_activated = self.task_guard_activated = False
    
    # querying the global guard activation
    def is_global_guard_activated(self):
        return self.global_guard_activated
    
    # querying the task guard activation    
    def is_task_guard_activated(self):
        return self.task_guard_activated

    def _build_tree_skeleton(self):
        self.do_task_selector = py_trees.composites.Selector('do_tasks_selector_branch', memory=True, children=[
                Print(f'One of the behaviours in the do_work_branch FAILED', print_tree=False),
                HandleTaskFailure('handle_task_failure', self.arm_commander),
            ],)
        self.cancellable_do_tasks_branch = py_trees.composites.Sequence (
            'cancellable_do_tasks_branch', memory=False, children = [
                HandleTaskCancelled('handle_cancel_task', self.arm_commander),
                self.do_task_selector,
            ]
        )
        do_tasks_trunk = py_trees.composites.Sequence(
            'do_work_trunk',
            memory=True,
            children=[
                py_trees.behaviours.CheckBlackboardVariableExists(name='check_task_exists', variable_name='task'),
                py_trees.behaviours.CheckBlackboardVariableValue(name='check_task_state', check=py_trees.common.ComparisonExpression(
                            variable='task.state', value=TaskStates.SUBMITTED, operator=operator.eq)),
                self.cancellable_do_tasks_branch,
            ],
        )
        self.task_guard = py_trees.decorators.EternalGuard('guarded_task_trunk', condition=self._get_task_guard_condition,
                                        child=do_tasks_trunk)
        self.root_sequence = py_trees.composites.Sequence('root_sequence', memory=True, children=[
            self.task_guard,
            ])
        self.global_guard = py_trees.decorators.EternalGuard('guarded_root_sequence', condition=self._get_global_guard_condition,
                                        child=self.root_sequence)
        self.the_root = BehaviourTree(self.global_guard) 
        return self.the_root

# ------------------------------------------
if __name__=='__main__':
    BasicTaskTreesManager()

