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
from task_trees.tools.logging_tools import logger

# -- The abstract class for implementing project specific Task for the TaskTreesManager
class BasicTask():
    """ It defines the essential parameters for the lifecycle management by the TaskTreesManager
    :meta private: 
    """
    def __init__(self, param=None):
        
        self.task_lock = threading.Lock()
        # task parameters
        self.name = self.__class__                          # the class name for identification of the task
        self.param = param                                  # arbitrary param of the task
        # task state
        self.state = TaskStates.STANDBY                     # the init state
        self.result = ''                                    # the task result, which will be assigned by the TaskTreesManager at completion
        self.commander_feedback:str = None
        # the cancel function of the task trees manager
        self._cancel_fn = None                              # internal use
        # the time submitted to the task trees manager
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
        :param cancel_fn: the cancel function provided by the Task Trees Manager
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

    def get_param(self):
        """ returns the param of the task

        :return: the param of the task
        :rtype: arbitrary, dependent on the subclass definition
        """
        return self.param
    
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
                logger.info(f'{type(self).__name__} (wait_for_completion): the waiting is finished (task_state: {self.state}) ...')                
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
# The generic Basic Task Trees Manager for managing the life cycle of the actions and behaviours for various tasks  
class BasicTaskTreesManager:
    """ subclass the BasicTaskTreesManager to develop a project specific task trees manager and its behaviour trees.
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
        logger.warning(f'{__class__.__name__}: SIGNINT signal received -> interrupt the tick-tock of the behaviour tree and shutdown the task trees manager')
        self.shutdown()
        sys.exit(0)

    # function for preventing the main thread from terminating the application
    def spin(self):
        while not self.to_shutdown:
            time.sleep(1.0)

    # shutdown the task trees manager
    def shutdown(self):
        """ Shutdown the task trees manager immediately
        """
        # terminate the behaviour tree tick-tock
        if self.bt is not None:
            self.bt.interrupt()
            self.bt.shutdown()
        else:
            logger.warning(f'{__class__.__name__}: attempted to shutdown a task trees manager with no behaviour tree installed')
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
    
    # allows a client application to submit a task to this task trees manager
    def submit_task(self, the_task:BasicTask) -> bool:
        """ The window for task submission.
        :param the_task: The task to be executed
        :type the_task: a subclass of robotarchi.BasicTask
        :return: True if the task trees manager is in the states that can accept the task submission 
        :rtype: bool
        """
        
        if self.the_blackboard.exists('task'):
            if self.the_blackboard.task.state in [TaskStates.SUBMITTED, TaskStates.WORKING]:
                logger.error(f'TaskTreesManager: submit a new task when a current task is SUBMITTED or WORKING')
                return False
        the_task._set_submitted(self._cancel_task)
        self.the_blackboard.set('task', the_task, overwrite=True) 
        return True
        
    # a utility function for rendering a the behaviour tree of this task trees manager as a dot file
    def display_tree(self):
        """ renders the behaviour tree of this task trees manager as a dot file
        """
        if self.bt is None:
            logger.error(f'{__class__.__name__}: attempted to display the bt of a task trees manager without one installed -> fix the subclass by assigning a behaviour tree to self.bt')
            raise AssertionError(f'A subclass of {__class__.__name__} has not assigned a behaviour tree to self.bt')         
        py_trees.display.render_dot_tree(self.bt.root)


# -----------------------------------------------------------------------------------------------------------
# The generic Task Trees Manager to support the framework of behaviour tree development 
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

    # shutdown the task trees manager
    def shutdown(self):
        """ Shutdown the task trees manager, including the behaviour tree and then terminate the program
        """
        self.arm_commander.abort_move(wait=True)
        super().shutdown()

    def _build_tree_skeleton(self):
        """ Internal function for building the skeleton for this tree manager
        :meta private:
        """
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
    
    def _define_named_poses(self, the_scene:Scene):
        """ Define the named poses in the arm commander as specified in the scene configuration file under the key'named_poses'

        :param the_scene: the scene configuration object of the Scene class
        """
        named_poses = the_scene.get_named_poses_of_root_as_dict()
        for name in named_poses:
            print(f'named_pose {name}: {named_poses[name]}')
            self.arm_commander.add_named_pose(name, named_poses[name])
            
    def _define_links(self, the_scene:Scene, link_type=None, include_list:list=None, ignore_list:list=None):
        """ Define the collision objects in the arm commander as specified in the scene configuration file

        :param the_scene: the scene configuration object of the Scene class
        :param include_list: the only frames to be included and subject to the ignore_list, defaults to None   
        :param link_type: the link_type to be included, defaults to None
        """
        if link_type is not None and type(link_type) is not list:
            link_type = [link_type]
        if ignore_list is None:
            ignore_list = []
        for scene_name in the_scene.list_scene_names():
            the_link = the_scene.get_link_of_scene(scene_name)
            if the_link is None:
                continue
            
            if link_type is not None and the_link.type not in link_type:
                continue
            if include_list is not None and scene_name not in include_list:
                continue 
            if scene_name in ignore_list:
                continue
            if the_link.type == 'box':
                self.arm_commander.add_box_to_scene(scene_name, the_link.dimensions, the_link.xyz, the_link.rpy, the_link.parent_frame)
            elif the_link.type == 'sphere':
                self.arm_commander.add_sphere_to_scene(scene_name, the_link.dimensions, the_link.xyz, the_link.parent_frame)
            elif the_link.type == 'object':
                self.arm_commander.add_object_to_scene(scene_name, the_link.model_file, the_link.dimensions, the_link.xyz, the_link.rpy, the_link.parent_frame)
            else:
                logger.warning(f'TaskTreesManager (_define_objects): unrecognize object type "{the_link.type}"')

    def _define_custom_frames(self, the_scene:Scene, include_list:list=None, ignore_list:list=None):
        """ Define the custom frames in the arm commander as specified in the scene configuration file

        :param the_scene: the scene configuration object of the Scene class
        :param include_list: the only frames to be included and subject to the ignore_list, defaults to None       
        :param ignore_list: the frames to be ignored, defaults to None
        """
        if ignore_list is None:
            ignore_list = []
        for frame_name in the_scene.list_frame_names():
            if include_list is not None and frame_name not in include_list:
                continue 
            if frame_name in ignore_list:
                continue
            the_frame_link = the_scene.get_link_of_frame(frame_name)
            self.arm_commander.add_custom_transform(frame_name, the_frame_link.xyz, the_frame_link.rpy, the_frame_link.parent_frame)
        
    def _set_initialize_branch(self, branch:Composite):
        """ Set or replace a branch to be the initialization branch of the behaviour tree of this manager

        :param branch: a branch type Composite of behaviour tree nodes
        """
        if self.num_initialize_branch > 0:
            logger.error(f'{__class__.__name__} (_set_initialize_branch): cannot set initialization branch more than once-> fix the subclass and avoid calling the function more then once')
            raise AssertionError(f'A parameter should not be None nor missing')  
        self.initialize_branch = py_trees.decorators.OneShot('init_oneshot_branch', policy = py_trees.common.OneShotPolicy.ON_COMPLETION, child=branch)
        self.root_sequence.insert_child(self.initialize_branch, 0)
        self.num_initialize_branch = 1
        
    def _add_priority_branch(self, branch:Composite):
        """ Add a branch to be a priority branch of the behaviour tree of this manager, the priority of this branch is lower than the previous ones

        :param branch: a branch type Composite of behaviour tree nodes
        """
        index = self.num_initialize_branch + self.num_priority_branch
        self.root_sequence.insert_child(branch, index)
        self.num_priority_branch += 1
    
    def _add_task_branch(self, branch:Composite, task_class:type=None):
        """ Add a branch to be a task branch of the behaviour tree of this manager

        :param branch: a branch type Composite of behaviour tree nodes
        :param task_class: the task class, which is a subclass of BasicTask, that is associated with the branch.
        """
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
        """ Save a graphical representation of the given branch of a behaviour tree to the target directory

        :param branch: the branch to be drawn, defaults to None which means the whole behaviour tree
        :param target_directory: the output directory, defaults to None
        """
        branch = self.bt.root if branch is None else branch
        if branch is None:
            return
        py_trees.display.render_dot_tree(branch, target_directory=target_directory)


# -----------------------------------------------------------------------------------------------------------
# The generic Guarded Task Trees Manager to support the framework of behaviour tree development 
# it is for managing the life cycle of the actions and behaviours for various tasks  
# the difference from TaskTreesManager is the option of setting conditions to invalidate the behaviour at every tick-tock
class GuardedTaskTreesManager(TaskTreesManager):
    """ subclass the TaskTreesManager to develop application specific behaviour trees that are structures to support
        the plug-in of tasks, timeout, and on-shot initialization branches 
    """
    def __init__(self, arm_commander:GeneralCommander, guard_reset=True):
        """The constructor for this class

        :param arm_commander: The commander to be used for the behaviours in the task tree
        :type arm_commander: GeneralCommander
        :param guard_reset: whether an activated guard needs reset to go back to unactivated state, defaults to True
        :type guard_reset: bool, optional
        """
        self.global_guard_activated = False
        self.task_guard_activated = False
        self.custom_global_guard_condition_fn = self.custom_task_guard_condition_fn = None        
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
        """ Set the function that returns the global guard condition

        :param global_guard_condition_fn: a function definition that returns a bool
        """
        if global_guard_condition_fn is not None and not hasattr(global_guard_condition_fn, '__call__'):
            raise AssertionError(f'GuardedTaskTreesManager: The parameter root_guard_condition is not a function')
        self.custom_global_guard_condition_fn = global_guard_condition_fn

    def set_task_guard_condition_fn(self, task_guard_condition_fn):
        """ Set the function that returns the task subtree guard condition

        :param task_guard_condition_fn: a function definition that returns a bool
        """
        if task_guard_condition_fn is not None and not hasattr(task_guard_condition_fn, '__call__'):
            raise AssertionError(f'GuardedTaskTreesManager: The parameter task_guard_condition is not a function')
        self.custom_task_guard_condition_fn = task_guard_condition_fn       
    
    # override the submit_task method
    def submit_task(self, the_task:BasicTask) -> bool:
        """ Submit a task to the task trees manager for execution

        :param the_task: the task to be execution
        :return: True if the submission is accepted
        """
        if self.global_guard_activated or self.task_guard_activated:
            logger.error(f'GuardedTaskTreesManager: submit a new task when the Guard is still activated -> call reset_guard before submitting tasks')            
            return False
        return BasicTaskTreesManager.submit_task(self, the_task)
    
    # reset the guard activation
    def reset_guard(self):
        """ Reset the guard that has been switched on
        """
        if (self.custom_global_guard_condition_fn is not None and self.custom_global_guard_condition_fn() == False) or \
            (self.custom_task_guard_condition_fn is not None and self.custom_task_guard_condition_fn() == False):
            logger.warning(f'{__class__.__name__} (reset_guard): Unable to reset guard activation status because the guard condition is still False')
            return
        self.global_guard_activated = self.task_guard_activated = False
    
    # querying the global guard activation
    def is_global_guard_activated(self):
        """ Returns True if the global guard has been activated
        """
        return self.global_guard_activated
    
    # querying the task guard activation    
    def is_task_guard_activated(self):
        """ Returns True if the task subtree guard has been activated
        """
        return self.task_guard_activated

    # internal function for building the tree skeleton
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

