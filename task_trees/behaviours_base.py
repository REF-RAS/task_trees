# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import numbers
from enum import Enum
from collections import namedtuple, defaultdict
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

# robot control module
from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderStates
from task_trees.states import TaskStates
from task_trees.task_scene import Scene
from tools.logging_tools import logger
# -----------------------------------------------------------------
# The base behaviours as an extension to PyTrees for working
# with other components in a robot arm manipulation application

# The base Behaviour class that supports the implementation of Conditional Behaviours, of which the
# behaviors are separated into condition and action
class ConditionalBehaviour(Behaviour):
    """ The base class that extends PyTrees Behaviour with support for the conditional behaviour model.
        The model significantly improves the reusability of behaviours and the readability of their
        roles in a behaviour tree.
        This class should not be instantiated
    """ 

    # the policies defined for the PyTree status to return if the condition is False 
    class Policies(Enum):
        SUCCESS_IF_FALSE_POLICY = 0  # suitable for use under a Sequence or Parallel composite node  
        FAILURE_IF_FALSE_POLICY = 1  # suitable for use under a Selector composite node
    # the namedtuple for better readability of an internal data structure 
    ConditionSpec = namedtuple('ConditionSpec', ['fn', 'inverted', 'kwargs']) 
    # the constructor
    def __init__(self, name:str, condition_fn=True, condition_policy:Policies=None):
        """ the constructor for this abstract class
        :param name: behaviour name
        :type name: str
        :param condition_fn: specification of condition, defaults to True
        :type condition_fn: bool, function, dict or list containing the aforementioned, optional
        :param condition_policy: the PyTrees status to return when the condition is evaluated to False, defaults to auto-detect the parent class
        :type condition_policy: Policies, optional
        """

        super(ConditionalBehaviour, self).__init__(name)
        self.condition_result = None
        self.condition_list = self._preprocess_condition_fn(condition_fn)
        self.condition_policy:ConditionalBehaviour.Policies = condition_policy
                
        global evaluation_records 
        evaluation_records = defaultdict(lambda: None)
        # logger.info(f'Condition list: {self.__class__.__name__} {self.condition_list}')

    # internal function that checks the syntax of condition_fn and if it is valid, convert it into a list for more uniform and efficient evaluation
    # the list is returned to the constructor and saved to an instance variable
    def _preprocess_condition_fn(self, condition_fn):
        condition_fn_type = type(condition_fn)
        # consider the 4 acceptable format of condition_fn
        if condition_fn_type == bool:  # a primitive bool
            self.condition_result = condition_fn
            return [condition_fn]
        elif hasattr(condition_fn, '__call__'):  # a function def
            return [ConditionalBehaviour.ConditionSpec(condition_fn, False, None)]
        elif condition_fn_type == dict:
            return [self._validate_condition_fn_dict(condition_fn)]
        elif condition_fn_type in [list, tuple]:
            condition_fn_list = []
            # if it is a list, iterate and process each element
            for i, item in enumerate(condition_fn):
                if hasattr(item, '__call__'):  # a function def
                    condition_fn_list.append(ConditionalBehaviour.ConditionSpec(item, False, None))
                elif type(item) == dict:
                    condition_fn_list.append(self._validate_condition_fn_dict(item))
                else:
                    logger.error(f'{__class__.__name__} ({self.name}): the type for item index {i} as part of the parameter', 
                                     '(condition_fn or its late derivation) must be either a bool, a function, a dict, or a list/tuple',
                                     ' -> fix the condition_fn specification at the behaviour creation')
                    raise ValueError('The value of the parameter is invalid')
            return condition_fn_list
        else:
            logger.error(f'{__class__.__name__} ({self.name}): the type of the parameter (condition_fn) must be ',
                                        'either a bool, a function, a dict, or a list/tuple',
                                        ' -> fix the condition_fn specification at the behaviour creation')
            raise ValueError('The value of the parameter is invalid')           
    
    # internal function that validates the format of the dict for condition specification
    # in particular it should contains one of the keys: '_fn', '_not_fn"
    def _validate_condition_fn_dict(self, dict_item):
        if '_fn' in dict_item and '_not_fn' in dict_item:
            logger.error(f'{__class__.__name__} ({self.name}): the parameter (condition_fn) is a dict and should include ',
                         'only one the two keys \'_fn\' or \'_not_fn\' -> keep only one of these in one item in the specification list')
            raise ValueError('The value of the parameter is invalid') 
        if '_fn' in dict_item:    
            fn, inverted = dict_item['_fn'], False
            del dict_item['_fn']
        elif '_not_fn' in dict_item:
            fn, inverted = dict_item['_not_fn'], True
            del dict_item['_not_fn']
        else:
            logger.error(f'{__class__.__name__} ({self.name}): the parameter (condition_fn) is a dict and should include ',
                         'at least one the two keys \'_fn\' or \'_not_fn\' -> add one of these in one item in the specification list')
            raise ValueError('The value of the parameter is invalid') 
        # ensure the dict_item contains at laeat one key-value pair
        if len(dict_item) >= 1:
            return ConditionalBehaviour.ConditionSpec(fn, inverted, dict_item)
        return ConditionalBehaviour.ConditionSpec(fn, inverted, None)

    # internal function, called by the update function
    def _is_condition_satified(self):
        if self.condition_result is not None:
            return self.condition_result
        for spec in self.condition_list:
            if spec.kwargs is None:
                result = spec.fn()
            else:
                try:
                    result = spec.fn(**(spec.kwargs))
                except TypeError as e:
                    logger.error(f'{type(self).__name__} Error in specifying the condition_fn for a behaviour: {e}')
                    raise
            result = not result if spec.inverted else result
            # print result when first evaluated or result changed
            arguments = ' '.join([str(v) for v in spec.kwargs.values()]) if spec.kwargs is not None else ' '
            if (spec.fn, arguments) not in evaluation_records or evaluation_records[(spec.fn, arguments)] != result:
                modifier = 'NOT ' if spec.inverted else ''
                logger.info(f'{type(self).__name__} ({self.name}) condition "{modifier}{spec.fn.__name__} ({arguments})" changed to: {result}')
                evaluation_records[(spec.fn, arguments)] = result                
            if not result:
                return False 
        return True
    
    # specialized update function for ConditionalBehaviour, contains logic for condition execution of the behaviour 
    def update(self) -> Status:
        """ Specialized update function for ConditionalBehaviour. Maybe overrided by subclasses and in such case checks the condition
            using the function _is_condition_satisfied()
            
        :return: the new PyTrees status
        :rtype: Status
        """
        need_to_run_behaviour = self._is_condition_satified()
        if need_to_run_behaviour:
            return self.update_if_true()
        elif self.condition_policy is not None:
            return_state = Status.SUCCESS if self.condition_policy == ConditionalBehaviour.SUCCESS_IF_FALSE_POLICY else Status.FAILURE
            return return_state
        else:
            if self.parent is not None:
                if isinstance(self.parent, py_trees.composites.Selector):
                    return Status.FAILURE
            return Status.SUCCESS
    
    # a placeholder function to be overridden in a subclass
    def update_if_true(self) -> Status:
        """ A placeholder function that should be overridden in a subclass
        :return: The new PyTrees status
        :rtype: Status
        """

        # no action defined here and therefore return FAILURE
        return Status.FAILURE    
      

# The base Behaviour class supporting the implementation of behaviours that uses the General Commander
# The class has included logic for managing the states of the General Commander
class ConditionalCommanderBehaviour(ConditionalBehaviour):
    """ The base Behaviour class supporting the implementation of Behaviour subclasses that uses the General Commander, and it
        has included logic for managing the states of the General Commander
    """
    def __init__(self, name:str, condition_fn=True, condition_policy=None, 
                 arm_commander:GeneralCommander=None, reference_frame=None):
        """ the constructor for this abstract class
        :param name: behaviour name
        :type name: str
        :param condition_fn: specification of condition, defaults to True
        :type condition_fn: bool, function, dict or list containing the aforementioned, optional
        :param condition_policy: the PyTrees status to return when the condition is evaluated to False, defaults to auto-detect the parent class
        :type condition_policy: Policies, optional
        :param arm_commander: the general commander to use in this behavour, defaults to None
        :type arm_commander: GeneralCommander
        """
        super(ConditionalCommanderBehaviour, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy)
        if arm_commander is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (arm_commander) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing') 
        self.arm_commander:GeneralCommander = arm_commander
        self.commander_state = self.arm_commander.get_commander_state()
        self.reference_frame = reference_frame if reference_frame is not None else self.arm_commander.WORLD_REFERENCE_LINK
        # define two keys that supports the operation of the subclasses of ConditionalCommanderBehaviour
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='task', access=py_trees.common.Access.WRITE)  # store the current task of type BasicTask or subclass
 
    @staticmethod
    def _bind_reference_frame(reference_frame):
        # process the reference_frame, and calls it for the actual reference frame if it is a function
        if reference_frame is not None and hasattr(reference_frame, '__call__'):
            return reference_frame()
        return reference_frame

    # A concrete implementation of the initialize function of PyTrees Behaviour
    def initialise(self):
        # logger.info(f'ConditionalMoveitBehaviour {self.name} initialize {self.arm_commander.get_commander_state()}')
        self.arm_commander.reset_state()    
                
    # A concrete implementation of the initialize function of PyTrees Behaviour           
    def update(self):
        self.commander_state = self.arm_commander.get_commander_state()
        # handle each state of the General Commander accordingly
        if self.commander_state == GeneralCommanderStates.READY:
            need_to_run_behaviour = self._is_condition_satified()
            # logger.info(f'{self.__class__.__name__} condition {self.name}: {need_to_run_behaviour}')
            if need_to_run_behaviour:
                return self.update_when_ready()
            elif self.condition_policy is not None:
                return_state = Status.SUCCESS if self.condition_policy == ConditionalBehaviour.SUCCESS_IF_FALSE_POLICY else Status.FAILURE
                return return_state
            else:
                if self.parent is not None:
                    if isinstance(self.parent, py_trees.composites.Selector):
                        return Status.FAILURE
                return Status.SUCCESS       
        elif self.commander_state == GeneralCommanderStates.BUSY:
            return self.update_when_busy()
        elif self.commander_state == GeneralCommanderStates.SUCCEEDED:
            self.tidy_up()
            return Status.SUCCESS
        elif self.commander_state == GeneralCommanderStates.ABORTED:
            logger.error(f'ConditionalCommanderBehaviour handles task aborted')
            if self.the_blackboard.exists('task'):
                self.the_blackboard.set('task.state', TaskStates.ABORTED)        
            self.tidy_up()
            return Status.FAILURE
        elif self.commander_state == GeneralCommanderStates.ERROR:
            logger.error(f'ConditionalCommanderBehaviour handles task error')
            if self.the_blackboard.exists('task'):
                self.the_blackboard.set('task.state', TaskStates.FAILED)        
            self.tidy_up()
            return Status.FAILURE         
        else:                
            return Status.SUCCESS
    # a placeholder function to be overridden in a subclass
    def update_when_ready(self):
        # no action defined here and therefore return FAILURE
        return Status.FAILURE
    # a placeholder function to be overridden in a subclass
    def update_when_busy(self):
        # no action defined here and therefore return RUNNING       
        return Status.RUNNING
    # a placeholder function to be overridden in a subclass
    def tidy_up(self):
        # logger.info(f'ConditionalCommanderBehaviour tidy_up {self.name} abort move')
        self.arm_commander.abort_move()
    # A concrete implementation of the initialize function of PyTrees Behaviour 
    def terminate(self, new_status):
        # logger.info(f'*** ConditionalCommanderBehaviour {self.name}: terminate {new_status} {self.arm_commander.get_commander_state()}')
        if self.arm_commander.get_commander_state() == GeneralCommanderStates.BUSY:
            # logger.info(f'ConditionalCommanderBehaviour terminate {self.name} abort move')
            self.arm_commander.abort_move()

class SceneConditionalCommanderBehaviour(ConditionalCommanderBehaviour):
    """ The base Behaviour class supporting the implementation of Behaviour subclasses that uses the General Commander
        and the Scene Configuration
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                 scene=None, reference_frame=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param named_pose: the named pose
        :type named_pose: a string or a function that returns the named_pose as a string
        """
        super(SceneConditionalCommanderBehaviour, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                                                 arm_commander=arm_commander, reference_frame=reference_frame)
        if scene is None:
            logger.warning(f'{__class__.__name__} ({self.name}): no scene model is provided -> acceptable if logical pose is not involved in this behaviour')
        self.the_scene:Scene = scene

    def _is_physical_target(self, target) -> bool:
        return not (target is None or type(target) not in [list, tuple] or not all([(isinstance(n, numbers.Number) or n is None) for n in target]) or len(target) != 3)

    # the method containing the main functionality of this class providing to the subclasses
    # it processes a 'composite_target' and returns a physical target (a list of three numbers) 
    def compute_physical_target(self, composite_target, logical_lookup_fn=None, default_target=None) -> list:
        if composite_target is None:
            return default_target
        if type(composite_target) in [list, tuple]:
        # test if it is a list of all numbers, warp it
            if all([(isinstance(n, numbers.Number) or n is None) for n in composite_target]):
                composite_target_list = [composite_target]
            else: # if the list is a composite, leave it as it is
                composite_target_list = composite_target
        else:
        # add the item (which can be a function or a string logical xyz), wrap it in a list for unified processing of a compositional list
            composite_target_list = [composite_target]
        # add the default xyz to the end of the compositional list
        if default_target is not None:
            composite_target_list.append(default_target)
        # initialize the placeholder
        physical = [None] * 3
        # iterate through each item in the compositional list
        for item in composite_target_list:
            # - the item is a function, call it to obtain a numeric xyz as a list or a string logical xyz
            if hasattr(item, '__call__'):
                item = item()
            if not self._is_physical_target(item):
                if logical_lookup_fn is not None:
                    item = logical_lookup_fn(item)    # the function that converts a logical target to a physical target of a scene 
                else:
                    logger.error(f'{__class__.__name__} ({self.name}): parameter (scene) is None for processing a logical value {item} -> fix the missing scene parameter at behaviour creation')  
                    raise AssertionError(f'A parameter should not be None nor missing')     
            # - the item should be a list of three numbers
            if not self._is_physical_target(item):
                logger.error(f'{__class__.__name__} ({self.name}): invalid item "{item}" in the omposite target -> fix the target specification (or its composition) at behaviour creation')       
                raise AssertionError(f'A target position or orientation is wrongly specified')  
            # - merge the item into the placeholder if the corresponding list element is None
            for i, _ in enumerate(physical):
                if physical[i] is None:
                    physical[i] = item[i]
        return physical    

# A utility Behaviour that handles task failure situation that has happened in a subclass of CommanderBehaviour or 
# ConditionalCommanderBehaviour
class HandleTaskFailure(Behaviour):
    """ A utility Behaviour that handles task failure situation that has happened in a subclass of CommanderBehaviour or 
        ConditionalCommanderBehaviour
    """
    def __init__(self, name:str, arm_commander=None):
        """ A utility Behaviour that handles task failure situation that has happened in a subclass of CommanderBehaviour or 
        ConditionalCommanderBehaviour
        :param name: behaviour name
        :type name: str
        :param arm_commander: the general commander to use in this behavour, defaults to None
        :type arm_commander: GeneralCommander
        """
        super(HandleTaskFailure, self).__init__(name)
        self.arm_commander:GeneralCommander = arm_commander
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='task', access=py_trees.common.Access.READ)    
          
    # A concrete implementation of the initialize function of PyTrees Behaviour     
    def update(self):
        the_task = self.the_blackboard.task
        commander_state = self.arm_commander.get_commander_state()
        if commander_state == GeneralCommanderStates.READY:  
            # the commander has no error, so check system error due to behaviour tree definition errors
            # possible reasons include invalid task name or parameters
            the_task.update_state(TaskStates.INVALID)
            the_task.result = f'{GeneralCommanderStates.ABORTED}: Invalid behaviour trees, parameters, or unknwon task'
        elif commander_state in [GeneralCommanderStates.ERROR]:
            the_task.update_state(TaskStates.FAILED)
            the_task.result = f'{commander_state}: {commander_state.message}'
        elif commander_state ==  GeneralCommanderStates.ABORTED:
            the_task.update_state(TaskStates.FAILED)            
        else:
            # this failure is reported by a behaviour in the tree, should have written the result, and disregard the commander
            the_task.update_state(TaskStates.FAILED) 

        logger.error(f'HandleTaskFailure: {self.the_blackboard.task.state} {self.the_blackboard.task.result} abort move and return FAILURE')
        self.arm_commander.abort_move()
        self.the_blackboard.unset('task')
        return Status.FAILURE
    
    # A concrete implementation of the initialize function of PyTrees Behaviour
    def terminate(self, new_status):
        pass
        # logger.info(f'HandleTaskFailure terminate {self.name} abort move')
        # self.arm_commander.abort_move()

# A utility Behaviour that handles task cancelled placed after the EternalGuard (with FailureIsSuccess)
class HandleTaskCancelled(Behaviour):
    """ A utility Behaviour that handles task cancelled placed after the EternalGuard (with FailureIsSuccess)
    """
    def __init__(self, name:str, arm_commander=None):
        """ A utility Behaviour that handles task cancelled placed after the EternalGuard (with FailureIsSuccess)
        :param name: behaviour name
        :type name: str
        :param arm_commander: the general commander to use in this behavour, defaults to None
        :type arm_commander: GeneralCommander
        """
        super(HandleTaskCancelled, self).__init__(name)
        self.arm_commander:GeneralCommander = arm_commander
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='task', access=py_trees.common.Access.READ)    
          
    # A concrete implementation of the initialize function of PyTrees Behaviour     
    def update(self):
        # the invalidation should result in GeneralCommander being ABORTED
        the_task = self.the_blackboard.task
        commander_state = self.arm_commander.get_commander_state()
        # logger.info(f'HandleTaskCancelled commander state: {commander_state} task state: {the_task.state}')
        if the_task.state == TaskStates.CANCELLED:
            the_task.state = TaskStates.ABORTED
            the_task.result = 'TASK_CANCELLED_BY_CLIENT'
            logger.error(f'HandleTaskCancelled: return FAILURE: {the_task.result}')
            self.the_blackboard.unset('task')
            self.stop(Status.FAILURE)
            return Status.FAILURE
        else:
            return Status.SUCCESS
    
    # A concrete implementation of the initialize function of PyTrees Behaviour
    def terminate(self, new_status):
        # logger.info(f'HandleTaskCancelled terminate {self.name} abort move')
        if new_status == Status.FAILURE:
            self.arm_commander.abort_move()


# A utility Behaviour used for tracing and debugging. It prints a string message from a constant or a function that returns a string
# and optionally the status of the subtree rooted at the parent of this behaviour.
class Print(Behaviour):
    """ a utility Behaviour used for tracing and debugging. It prints a string message from a constant or a function that returns a string
        and optionally the status of the subtree rooted at the parent of this behaviour.
    """
    def __init__(self, msg, print_tree=False):
        super(Print, self).__init__('print_behaviour')
        self.msg = msg
        self.print_tree = print_tree
        
    # A concrete implementation of the initialize function of PyTrees Behaviour    
    def update(self):
        # if the msg is a function, call the function for the string message
        if hasattr(self.msg, '__call__'):
            msg = self.msg()
        else:
            msg = self.msg
        # output the message
        logger.info(f'{self.name}: {msg}')
        # print the ascii representation of the behaviour tree rooted at the parent of this behaviour  
        if self.print_tree:
            logger.info(f'{py_trees.display.ascii_tree(self.parent)}')
        # this is a tricky part - the behaviour is supposed to be 'transparent' in a sequence of behaviours
        # but the return status has to be SUCCESS if the parent is a Sequence and FAILURE if the parent is a Selector
        if self.parent is not None:
            if isinstance(self.parent, py_trees.composites.Selector):
                return Status.FAILURE
        return Status.SUCCESS  
    
# A utility Behaviour used for tracing and debugging. It prints the pose of the end-effector in the world frame and the joint values
class PrintPose(Behaviour):
    """ a utility Behaviour used for tracing and debugging. It prints the pose of the end-effector in the world frame and the joint values
    """
    def __init__(self, arm_commander:GeneralCommander=None):
        super(PrintPose, self).__init__('print_pose_behaviour')
        self.arm_commander:GeneralCommander = arm_commander
    # A concrete implementation of the initialize function of PyTrees Behaviour    
    def update(self):
        logger.info(f'- Print the pose of the end-effector and joint values')
        logger.info(f'- xyzrpy: {self.arm_commander.pose_in_frame_as_xyzrpy()}')
        logger.info(f'- joints: {self.arm_commander.current_joint_positons_as_list()}')
        return Status.SUCCESS

# A utility Behaviour used for tracing and debugging. It prints the pose of the end-effector in the reference frames
# of a collision object or all objects
class PrintPosesInFrame(PrintPose):
    """ A utility Behaviour used for tracing and debugging. It prints a string message from a constant or a function that returns a string
        and optionally the status of the subtree rooted at the parent of this behaviour.
    """
    def __init__(self, arm_commander:GeneralCommander=None, scene:Scene=None, reference_frame:str=None):
        super(PrintPosesInFrame, self).__init__('print_pose_behaviour', arm_commander=arm_commander)
        self.reference_frame = reference_frame
        self.the_scene:Scene = scene
    # A concrete implementation of the initialize function of PyTrees Behaviour    
    def update(self):
        super().update()
        # if the msg is a function, call the function for the string message
        if hasattr(self.reference_frame, '__call__'):
            binded_reference_frame = self.reference_frame()
        else:
            binded_reference_frame = self.reference_frame
        if binded_reference_frame is None:
            binded_reference_frame = self.the_scene.list_object_names()
        else:
            binded_reference_frame = [binded_reference_frame]
        logger.info(f'- PrintPose of the end-effector')
        for frame in binded_reference_frame:
            logger.info(f'- {frame}: {self.arm_commander.pose_in_frame_as_xyzrpy(reference_frame=frame)}')
        return Status.SUCCESS

class SimAttachObject(ConditionalCommanderBehaviour):
    """ This behaviour attachs a collision object to the end effector
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                 object_name=None, post_fn=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        """
        super(SimAttachObject, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                              arm_commander=arm_commander)
        if object_name is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (object_name) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing') 
        self.object_name = object_name
        self.post_fn = post_fn
        if self.post_fn is not None and not hasattr(self.post_fn, '__call__'):
            logger.warning(f'{__class__.__name__} ({self.name}): parameter (post_fn) is not a function -> fix the parameter at behaviour construction')

    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):           
        self.arm_commander.attach_object_to_end_effector(self.object_name)
        if self.post_fn is not None:
            self.post_fn()
        return Status.SUCCESS
    
        
class SimDetachObject(ConditionalCommanderBehaviour):
    """ This behaviour detachs a collision object from the behaviour
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                 object_name=None, post_fn=None, to_remove=False):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        """
        super(SimDetachObject, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                              arm_commander=arm_commander)
        if object_name is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (object_name) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing') 
        self.object_name = object_name
        self.the_blackboard.register_key('the_object', access=py_trees.common.Access.WRITE)
        self.to_remove = to_remove
        self.post_fn = post_fn
        if self.post_fn is not None and not hasattr(self.post_fn, '__call__'):
            logger.warning(f'{__class__.__name__} ({self.name}): parameter (post_fn) is not a function -> fix the parameter at behaviour construction')

    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):       
        # send the command to the General Commander in an asynchronous manner        
        self.arm_commander.detach_object_from_end_effector(self.object_name)
        if self.post_fn is not None:
            self.post_fn()
        if self.to_remove:
            self.arm_commander.remove_object(self.object_name)
        return Status.SUCCESS