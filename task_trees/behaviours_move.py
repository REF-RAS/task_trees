# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

from py_trees.common import Status
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
# robot control module
from task_trees.behaviours_base import ConditionalCommanderBehaviour, SceneConditionalCommanderBehaviour, PrintPose
from tools.logging_tools import logger

# ----------------------------------------------------------------------
# Move Behaviour Classes for robot arm manipulation applications

class DoMoveNamedPose(SceneConditionalCommanderBehaviour):
    """ This behaviour moves the robot arm to a named pose defined by joint-values
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, scene=None, named_pose=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param named_pose: the named pose
        :type named_pose: a string or a function that returns the named_pose as a string
        """
        super(DoMoveNamedPose, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                              arm_commander=arm_commander, scene=scene)
        if named_pose is None or scene is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (named_pose or scene) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')
        self.the_scene = scene
        self.named_pose = named_pose
    # the concrete implementation of the logic when the General Commander is READY
    def update_when_ready(self):
        # obtain the named pose, and if it is a function, call the function to get the string
        named_pose = self.named_pose
        if hasattr(named_pose, '__call__'):
            named_pose = named_pose()
        the_scene = self.the_scene
        # validate the named_pose
        if named_pose is None or type(named_pose) != str or not the_scene.exists_config(named_pose):
            logger.error(f'DoMoveNamedPose ({self.name}): invalid named pose "{named_pose}" parameter')
            return Status.FAILURE
        # send the command to the General Commander in an asynchronous manner
        self.arm_commander.move_to_named_pose(named_pose, wait=False)
        logger.info(f'DoMoveNamedPose ({self.name}): started move to named pose: {named_pose}')         
        return Status.RUNNING

class DoMoveJointPose(ConditionalCommanderBehaviour):
    """ This behaviour moves the robot arm to a pose defined joint-values
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, target_joint_pose=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param target_joint_pose: the target joint pose
        :type target_joint_pose: a list of joint values or a function that returns the list
        """
        super(DoMoveJointPose, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                              arm_commander=arm_commander)
        if target_joint_pose is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_joint_pose) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')
        self.joint_pose = target_joint_pose
    # the concrete implementation of the logic when the General Commander is READY
    def update_when_ready(self):
        # obtain the named pose, and if it is a function, call the function to get the string
        joint_pose = self.joint_pose
        if hasattr(joint_pose, '__call__'):
            joint_pose = joint_pose()
        # validate the named_pose
        if joint_pose is None or type(joint_pose) != list:
            logger.error(f'DoMoveJointPose ({self.name}): invalid joint pose "{joint_pose}" parameter')
            return Status.FAILURE
        # send the command to the General Commander in an asynchronous manner
        self.arm_commander.move_to_joint_pose(joint_pose, wait=False)
        logger.info(f'DoMoveJointPose ({self.name}): started move to joint pose: {joint_pose}')         
        return Status.RUNNING

class DoMovePose(ConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector to a pose defined in a reference frame according to a path planner. 
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                target_pose=None, reference_frame=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param target_pose: the target pose
        :type target_pose: a list of 6 numbers, 7 numbers, Pose, or PoseStamped, or a function that returns the above
        :param reference_frame: the reference_frame
        :type reference_frame: a string representing the reference_frame      
        """
        super(DoMovePose, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                         arm_commander=arm_commander, reference_frame=reference_frame)
        if target_pose is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_pose) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing') 
        if not hasattr(target_pose, '__call__') and type(target_pose) not in [Pose, PoseStamped, list, tuple]:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_pose) is not of an acceptable type -> use Pose, PoseStamped, 6-list or 7-list at behaviour construction')
            raise AssertionError(f'The paramete (target_pose) should be of the acceptable type')  
        self.target_pose = target_pose
        
    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        # resolve the late binding target pose and reference frame
        binded_reference_frame = self._bind_reference_frame(self.reference_frame)
        if hasattr(self.target_pose, '__call__'):
            binded_target_pose = self.target_pose()
        else:
            binded_target_pose = self.target_pose
        self.arm_commander.move_to_pose(binded_target_pose, binded_reference_frame, wait=False)
        logger.info(f'DoMovePose ({self.name}): started move to pose: {binded_target_pose} in reference frame "{binded_reference_frame}"')   
        return Status.RUNNING
    
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.arm_commander.clear_path_constraints()

class DoMoveXYZ(SceneConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector to a pose defined in the frame of the work area in a cartesian manner. 
        The roll and pitch components of the orientation of the end-effector is fixed.
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene=None, target_xyz=None, reference_frame=None, cartesian=True, constraint_fn=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param the_scene: the scene model for the handling of logical positions specified in the target xyz
        :type the_scene: Scene
        :param target_xyz: the target position, which can be specified in several ways
        :type target_xyz: a composite that can be a list of 3 numbers, a function, a string, or a list of the aforementioned
        :param reference_frame: the reference_frame
        :type reference_frame: a string representing the reference_frame      
        """
        super(DoMoveXYZ, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                        arm_commander=arm_commander, scene=scene, reference_frame=reference_frame)
        if target_xyz is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_xyz) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')
        if scene is None:
            logger.warning(f'{__class__.__name__} ({self.name}): no scene model is provided -> acceptable if logical pose is not involved in this behaviour')
        self.target_xyz = target_xyz
        self.cartesian = cartesian
        self.constraint_fn = constraint_fn
        
    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        # evaluate physical target xyz
        if self.the_scene is None:
             xyz = self.compute_physical_target(self.target_xyz) 
        else:
            xyz = self.compute_physical_target(self.target_xyz, self.the_scene.query_config) 
        if self.target_xyz is None or xyz is None:
            logger.error(f'DoMoveXYZ ({self.name}): invalid position xyz parameter {xyz}')
            return Status.FAILURE 
        # add constraint returned by constraint_fn is given
        if self.constraint_fn is not None and hasattr(self.constraint_fn, '__call__'):
            self.arm_commander.add_path_constraints(self.constraint_fn())
        # send command
        binded_reference_frame = self._bind_reference_frame(self.reference_frame)
        logger.info(f'DoMoveXYZ ({self.name}): started move to pose: {xyz} in reference frame "{binded_reference_frame}"')         
        self.arm_commander.move_to_position(x=xyz[0], y=xyz[1], z=xyz[2], cartesian=self.cartesian, reference_frame=binded_reference_frame, wait=False)
        return Status.RUNNING
    
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.arm_commander.clear_path_constraints()
        
class DoMoveXYZRPY(SceneConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector to a pose defined in the frame of the work area in a cartesian manner. 
        The roll and pitch components of the orientation of the end-effector is fixed.
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene=None, target_xyz=None, target_rpy=None, reference_frame=None, constraint_fn=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param the_scene: the scene model for the handling of logical positions specified in the target xyz
        :type the_scene: Scene
        :param xyz: the target position, which can be specified in a 
        :type xyz: a string representing the z_level_pose as defined in the task scene
        :param reference_frame: the reference_frame
        :type reference_frame: a string representing the reference_frame      
        """
        super(DoMoveXYZRPY, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                           arm_commander=arm_commander, scene=scene, reference_frame=reference_frame)
        if target_xyz is None or target_rpy is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_xyz or target_rpy) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')     
        if scene is None:
            logger.warning(f'{__class__.__name__} ({self.name}): no scene model is provided -> acceptable if logical pose is not involved in this behaviour')
        self.target_xyz = target_xyz
        self.target_rpy = target_rpy
        self.constraint_fn = constraint_fn
        
    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        # evaluate physical target xyz
        binded_reference_frame = self._bind_reference_frame(self.reference_frame)
        current_xyz = self.arm_commander.pose_in_frame_as_xyzrpy(reference_frame=binded_reference_frame)
        # logger.info(f'DoMoveXYZRPY current pose: {current_xyz} of {binded_reference_frame}')
        if self.the_scene is None:
            xyz = self.compute_physical_target(self.target_xyz, None, default_target=current_xyz[:3])  
            rpy = self.compute_physical_target(self.target_rpy, None, default_target=current_xyz[3:])            
        else:
            xyz = self.compute_physical_target(self.target_xyz, self.the_scene.query_position_as_xyz, default_target=current_xyz[:3])  
            rpy = self.compute_physical_target(self.target_rpy, self.the_scene.query_rotation_as_rpy, default_target=current_xyz[3:]) 
        if xyz is None:
            logger.error(f'DoMoveXYZRPY ({self.name}): invalid position xyz parameter {xyz}')
            return Status.FAILURE 
        if rpy is None:
            logger.error(f'DoMoveXYZRPY ({self.name}): invalid position rpy parameter {rpy}')
            return Status.FAILURE         
        target_pose = xyz + rpy
        # add constraint returned by constraint_fn is given
        if self.constraint_fn is not None and hasattr(self.constraint_fn, '__call__'):
            self.arm_commander.add_path_constraints(self.constraint_fn())
        logger.info(f'DoMoveXYZRPY ({self.name}): started move to pose: {target_pose} in reference frame "{binded_reference_frame}"')   
        self.arm_commander.move_to_pose(target_pose, reference_frame=binded_reference_frame, wait=False)
        return Status.RUNNING
    
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.arm_commander.clear_path_constraints()

class DoMoveDisplaceXYZ(SceneConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector to a pose defined in the frame of the work area in a cartesian manner. 
        The roll and pitch components of the orientation of the end-effector is fixed.
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene=None, dxyz=None, reference_frame=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param the_scene: the scene model for the handling of logical positions specified in the target xyz
        :type the_scene: Scene
        :param dxyz: the displacement to move, as a list of 3 numbers representing dx, dy, and dz
        :type dxyz: a composite that can be a list of 3 numbers, a function, a string, or a list of the aforementioned
        :param reference_frame: the reference_frame
        :type reference_frame: a string representing the reference_frame      
        """
        super(DoMoveDisplaceXYZ, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                                arm_commander=arm_commander, scene=scene, reference_frame=reference_frame)
        if dxyz is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (dxyz) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')
        if scene is None:
            logger.warning(f'{__class__.__name__} ({self.name}): no scene model is provided -> acceptable if logical pose is not involved in this behaviour')
        self.dxyz = dxyz
        self.reference_frame = reference_frame
        
    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        # evaluate physical dxyz
        if self.the_scene is None:
            dxyz = self.compute_physical_target(self.dxyz) 
        else:
            dxyz = self.compute_physical_target(self.dxyz, self.the_scene.query_config) 
        if self.dxyz is None or dxyz is None:
            logger.error(f'DoMoveDisplaceXYZ ({self.name}): invalid position dxyz parameter {dxyz}')
            return Status.FAILURE
        # compute the target pose
        binded_reference_frame = self._bind_reference_frame(self.reference_frame)
        xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy(reference_frame=binded_reference_frame)
        xyzrpy[0] += 0 if dxyz[0] is None else dxyz[0]
        xyzrpy[1] += 0 if dxyz[1] is None else dxyz[1]
        xyzrpy[2] += 0 if dxyz[2] is None else dxyz[2]        
        # send command
        logger.info(f'DoMoveDisplaceXYZ ({self.name}): started move to pose: {dxyz} in reference frame "{binded_reference_frame}"')         
        self.arm_commander.move_to_position(x=xyzrpy[0], y=xyzrpy[1], z=xyzrpy[2], cartesian=True, reference_frame=binded_reference_frame, wait=False)
        return Status.RUNNING
    
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.arm_commander.clear_path_constraints()

class DoRotate(SceneConditionalCommanderBehaviour):
    """ This behaviour rotates the end-effector to a target rpy in a given reference frame. The position is fixed.
    """    
    def __init__(self, name, condition_fn, condition_policy=None, arm_commander=None, 
                 scene=None, target_rpy=None, reference_frame=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param rotation_pose: the logical pose describing the rotation as defined in the task scene
        :type rotation_pose: a string, or a function that returns the rotatio pose
        """
        super(DoRotate, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                       arm_commander=arm_commander, scene=scene, reference_frame=reference_frame)
        if target_rpy is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_rpy) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing') 
        if scene is None:
            logger.warning(f'{__class__.__name__} ({self.name}): no scene model is provided -> acceptable if logical pose is not involved in this behaviour') 
        self.target_rpy = target_rpy
    # the concrete implementation of the logic when the General Commander is READY 
    def update_when_ready(self):
        # evaluate physical target rpy
        if self.the_scene is None:
            target_rpy = self.compute_physical_target(self.target_rpy)      
        else:
            target_rpy = self.compute_physical_target(self.target_rpy, self.the_scene.query_rotation_as_rpy)

        if self.target_rpy is None or target_rpy is None:
            logger.error(f'DoRotate ({self.name}): invalid target_rpy parameter {self.target_rpy}')
            return Status.FAILURE 
        # send the command to the General Commander in an asynchronous manner  
        binded_reference_frame = self._bind_reference_frame(self.reference_frame)
        logger.info(f'DoRotate ({self.name}): started rotate to orientation: {target_rpy} in reference frame "{binded_reference_frame}"')          
        self.arm_commander.rotate_to_orientation(roll=target_rpy[0], pitch=target_rpy[1], yaw=target_rpy[2], reference_frame=binded_reference_frame, wait=False)
        return Status.RUNNING
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.arm_commander.clear_path_constraints()
    
class DoMoveMultiPose(SceneConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector to multiple poses defined in a reference frame in a cartesian manner. 
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene=None, target_poses=None, reference_frame=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param the_scene: the scene model for the handling of logical positions specified in the target xyz
        :type the_scene: Scene
        :param target_poses: a list of target poses, each of which can be one of the acceptable format of poses, and alternatively a function that returns a list of constant target poses
        :type target_poses: a list of poses, each of which the format is either a list of 6 numbers, 7 numbers, Pose, or PoseStamped, or instead a function returning a list of constant target poses
        :param reference_frame: the reference_frame
        :type reference_frame: a string representing the reference_frame      
        """
        super(DoMoveMultiPose, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                         arm_commander=arm_commander, scene=scene, reference_frame=reference_frame)
        if target_poses is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_poses) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing') 
        if not hasattr(target_poses, '__call__') and type(target_poses) not in [list, tuple]:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_poses) is not of an acceptable type -> make sure a function or a list of poses used at construction')
            raise AssertionError(f'The parameter (target_poses) should be of the acceptable type')  
        self.target_poses = target_poses
        
    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        # resolve the late binding target pose and reference frame
        binded_reference_frame = self._bind_reference_frame(self.reference_frame)
        if hasattr(self.target_poses, '__call__'):
            binded_target_poses = self.target_poses()
        else:
            binded_target_poses = self.target_poses
        self.arm_commander.move_to_multi_poses(binded_target_poses, binded_reference_frame, wait=False)
        logger.info(f'DoMoveMultiPose ({self.name}): started move to mutli-poses: {binded_target_poses} in reference frame "{binded_reference_frame}"')   
        return Status.RUNNING


class DoMoveMultiXYZ(SceneConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector to a pose defined in the frame of the work area in a cartesian manner. 
        The roll and pitch components of the orientation of the end-effector is fixed.
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene=None, target_xyz_list=None, reference_frame=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param the_scene: the scene model for the handling of logical positions specified in the target xyz
        :type the_scene: Scene
        :param target_xyz: the target position, which can be specified in several ways
        :type target_xyz: a composite that can be a list of 3 numbers, a function, a string, or a list of the aforementioned
        :param reference_frame: the reference_frame
        :type reference_frame: a string representing the reference_frame      
        """
        super(DoMoveMultiXYZ, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, 
                                        arm_commander=arm_commander, scene=scene, reference_frame=reference_frame)
        if target_xyz_list is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_xyz_list) is None -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')
        if type(target_xyz_list) not in [list, tuple]:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (target_xyz_list) is not a list nor tuple -> fix the value at behaviour construction')
            raise AssertionError(f'A parameter should not be None nor missing')        
        if scene is None:
            logger.warning(f'{__class__.__name__} ({self.name}): no scene model is provided -> acceptable if logical pose is not involved in this behaviour')
        self.target_xyz_list = target_xyz_list
        
    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        binded_xyz_list = []
        for target_xyz in self.target_xyz_list:
            # evaluate physical target xyz
            if self.the_scene is None:
                xyz = self.compute_physical_target(target_xyz) 
            else:
                xyz = self.compute_physical_target(target_xyz, self.the_scene.query_config) 
            if self.target_xyz_list is None or xyz is None:
                logger.error(f'DoMoveXYZ ({self.name}): invalid position xyz parameter {xyz}')
                return Status.FAILURE 
            binded_xyz_list.append(xyz)
        # send command
        binded_reference_frame = self._bind_reference_frame(self.reference_frame)
        logger.info(f'DoMoveMultiXYZ ({self.name}): started move to multi poses: {binded_xyz_list} in reference frame "{binded_reference_frame}"')         
        self.arm_commander.move_to_multi_positions(xyz_list=binded_xyz_list, reference_frame=binded_reference_frame, wait=False)
        return Status.RUNNING
    