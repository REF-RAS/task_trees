# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import yaml, os, time
from collections import defaultdict
import rospy, tf, tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform, Vector3, Quaternion
from visualization_msgs.msg import Marker

from task_trees.task_scene import Scene, ObjectConfig
import tools.rviz_tools as rviz_tools
from tools.logging_tools import logger
from tools.rviz_tools import RvizVisualizer
from tools.pose_tools import pose_to_xyzq, list_to_xyzq, pose_stamped_to_transform_stamped, list_to_pose_stamped
from tools.rospkg_tools import PackageFile

class SceneToRViz(RvizVisualizer):
    """ The helper class for managing visualization of scene configs in rviz
    """
    def __init__(self, the_scene:Scene, base_frame:str=None, publish_objects_transform=True, pub_period:float=0.1):
        # check if a ROS node has been initialized 
        node_uri = rospy.get_node_uri()
        if node_uri is None:
            rospy.init_node('scene_rviz_node', anonymous=False)
            rospy.logwarn(f'The SceneRviz is not yet initialized as a ROS node.')
            rospy.logwarn(f'It is now running as a standalone application after initializing its own node.')
            rospy.logwarn(f'-> If SceneRViz is part of an application, resolve this programming issue by adding the call at the start of the application.')
            rospy.logwarn(f'    rospy.init_node(\'my_node\', anonymous=False)')
        super(SceneToRViz, self).__init__()
        self.the_scene = the_scene
        self.base_frame = base_frame
        self.is_transform_publisher = publish_objects_transform
        self.is_world_publisher = False
        self.publishable_object_names = []
        # setup transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        if self.base_frame is None:
            self.base_frame = 'world'
            self.is_world_publisher = True
            logger.warning(f'SceneToRViz: no base_frame given, use default base frame "world"')
        # setup custom transforms
        self.custom_transform_object_dict = defaultdict(lambda: None)
        # setup timer
        self.timer_transform = rospy.Timer(rospy.Duration(pub_period), self._cb_timer_transform)
    
    # internal function: for publishing the transform regularly

    def _cb_timer_transform(self, event):
        if self.is_world_publisher:
            header = Header()
            header.frame_id = 'map'
            header.stamp = rospy.Time.now()
            transform_stamped = TransformStamped(header, self.base_frame, Transform(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1)))
            self.tf_broadcaster.sendTransform(transform_stamped)
            self.is_world_publisher = False
        if self.is_transform_publisher:
            self._pub_transform_all_objects()
        self._pub_custom_transform()

    # internal function: publish the transform of objects in the scene config subject to the list of publishable objects 
    def _pub_transform_all_objects(self):
        for object_name in the_scene.list_object_names():
            if self.publishable_object_names is None or object_name in self.publishable_object_names:
                the_object = the_scene.get_object_config(object_name)
                xyzrpy = the_object.xyz + the_object.rpy
                self._pub_transform_object(object_name, xyzrpy, the_object.frame)
                
    # internal function: publish the transform of custom objects
    def _pub_custom_transform(self):
        for the_object in self.custom_transform_object_dict.values():
            xyzrpy = the_object.xyz + the_object.rpy
            self._pub_transform_object(the_object.name, xyzrpy, the_object.frame)

    # internal function: publish the transform of a specific named object
    def _pub_transform_object(self, name, pose, frame=None):
        """ publish the transform of an object

        :param name: name of the object
        :type name: str
        :param pose: the pose of the object 
        :type pose: Pose, PoseStamped, list of 6 or 7
        :param frame: the frame against which the pose is defined, ignored if PoseStamped is provided, defaults to None
        :type frame: str, optional
        """
        frame = self.base_frame if frame is None else frame
        if type(pose) in [list, tuple]:
            pose_stamped = list_to_pose_stamped(pose, frame)
        elif type(pose) == Pose:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = pose
        elif type(pose) == PoseStamped:
            frame = pose.header.frame_id
            pose_stamped = pose
        else:
            rospy.logerr(f'{__class__.__name__}: parameter (pose) is not list of length 6 or 7 or a Pose object -> fix the parameter at behaviour construction')
            raise TypeError(f'A parameter is invalid')
        self.tf_broadcaster.sendTransform(pose_stamped_to_transform_stamped(pose_stamped, name))

        # try:
        #     # frame is ignored if pose is PoseStamped
        #     if type(pose) == Pose:
        #         xyzq = pose_to_xyzq(pose)
        #     elif type(pose) == PoseStamped:
        #         frame = pose.header.frame_id            
        #         xyzq = pose_to_xyzq(pose.pose)
        #     else:
        #         xyzq = list_to_xyzq(pose)
        # except Exception:
        #     logger.error(f'{__class__.__name__}: parameter (pose) is not list of length 6 or 7 or a Pose object -> fix the parameter at behaviour construction')
        #     raise
        # frame = self.base_frame if frame is None else frame
        # self.tf_pub.sendTransform(xyzq[:3], xyzq[3:], rospy.Time.now(), name, frame)

    # --------------------------------
    def set_publish_objects_transform(self, object_names:list=None):
        """ specify the names of the objects that are to be included in transform publish

        :param object_names: a list of object names to be included, defaults to None
        :type object_names: list, optional
        """
        if object_names is not None and type(object_names) not in [tuple, list]:
            logger.error(f'SceneToRViz (set_publish_transform): parameter (object_names) must be a list of strings or None')
            raise AssertionError('Parameter (object_names) type is invalid')
        self.publishable_object_names = object_names

    def display_bbox_regions(self, config_key:str, plane:str='xy', rgba=[1.0, 0.0, 1.0, 0.2]) -> None:
        """ display as a bbox 2d or 3d region specified by the key or set of keys under the config_key parameter along the plane

        :param config_key: the name of the config key in the scene configuration, which is either the key itself or a parent of keys
        :type config_key: str
        :param plane: 'xy', 'yz' or 'xz', default is 'xy', ignored if the bbox is 3d
        :type plane: str, optional
        :param rgba: the colour of the 2d or 3d region, defaults to [1.0, 0.0, 1.0, 0.2]
        :type rgba: list, optional
        """
        value = self.the_scene.query_config(config_key, None)
        if value is None:
            logger.warning(f'SceneToRViz (register_regions_as_bbox): Non-existent key {config_key}')
            return
        subscene = self.the_scene.get_subscene_of_config_name(config_key)
        reference_frame = subscene if subscene is not None else self.base_frame
        if plane is None:
            plane = 'xy' # default to xy
        if type(value) in [list, tuple] and len(value) in [4, 6]:
            value = {' ': value}
        if type(value) == dict:
            for id, key in enumerate(value):
                if type(value[key]) in [list, tuple]:
                    if len(value[key]) == 4:
                        marker = rviz_tools.create_axisplane_marker(f'{config_key}.{key}', id, value[key], 0.0, reference_frame, plane, rgba=rgba)
                        self.add_persistent_marker(marker)
                        continue
                    elif len(value[key]) == 6:
                        marker = rviz_tools.create_cube_marker_from_bbox(f'{config_key}.{key}', id, value[key], reference_frame, rgba=rgba)
                        self.add_persistent_marker(marker)
                        continue
                logger.warning(f'SceneToRViz (display_bbox_regions): region ({key}) bbox should be a list of 4 or 6')

    def display_rotations(self, config_key:str, arrow_length:float=1.0, rgba=[1.0, 0.0, 0.0, 0.8]) -> None:
        """ display as an arrow the rotation specified by config_key

        :param config_key: the name of the config key in the scene configuration, which is either the key itself or a parent of keys
        :type config_key: str
        :param arrow_length: length of the array, defaults to 1.0
        :type arrow_length: float, optional
        :param rgba: the color of the arrow, defaults to [1.0, 0.0, 1.0, 0.2]
        :type rgba: list, optional
        """
        value = self.the_scene.query_config(config_key, None)
        if value is None:
            logger.warning(f'SceneToRViz (register_positions): Non-existent key {config_key}')
            return 
        subscene = self.the_scene.get_subscene_of_config_name(config_key)
        reference_frame = subscene if subscene is not None else self.base_frame              
        if type(value) in [list, tuple] and len(value) == 3:
            value = {' ': value}
        if type(value) == dict:
            xyz = [0, 0, 0]
            for id, key in enumerate(value): 
                rpy = value[key]
                if type(rpy) in [list, tuple] and len(rpy) == 3 and not any(x == None for x in rpy):
                    marker = rviz_tools.create_arrow_marker(f'{config_key}.{key}', id, xyz + rpy, reference_frame, arrow_length, rgba=rgba)
                    self.add_persistent_marker(marker)
                    continue     
                logger.warning(f'SceneToRViz (display_rotations): rotation ({key}) should be a list of 3')
                

    def display_positions(self, config_key:str, rgba=[1.0, 0.0, 0.0, 0.8]) -> None:
        """ display one or more positions or partial definitions, that a sphere represents full specification, a line represents a position with a null
        value, and a plane represents a position with two null values 

        :param config_key: the name of the config key in the scene configuration, which is either the key itself or a parent of keys
        :type config_key: str
        :param rgba: the colour of the line or plane, defaults to [1.0, 0.0, 1.0, 0.2]
        :type rgba: list, optional
        """
        value = self.the_scene.query_config(config_key, None)
        if value is None:
            logger.warning(f'SceneToRViz (display_positions): Non-existent key {config_key}')
            return 
        subscene = self.the_scene.get_subscene_of_config_name(config_key)
        reference_frame = subscene if subscene is not None else self.base_frame              
        if type(value) == list and len(value) == 3:
            value = {' ': value}
        if type(value) == dict:
            for id, key in enumerate(value): 
                xyz = value[key]
                if type(xyz) not in [list, tuple] or len(xyz) != 3:
                    logger.warning(f'SceneToRViz (display_positions): position ({key}) should be a list of 3')
                    continue
                none_count = xyz.count(None)
                if none_count == 0:
                    marker = rviz_tools.create_sphere_marker(f'{config_key}.{key}', id, xyz, reference_frame, rgba=rgba)
                    self.add_persistent_marker(marker)
                elif none_count == 1:
                    index_of_none = xyz.index(None)
                    xyz1 = xyz.copy()
                    xyz2 = xyz.copy()
                    xyz1[index_of_none] = -1
                    xyz2[index_of_none] = 1
                    marker = rviz_tools.create_line_marker(f'{config_key}.{key}', id, xyz1, xyz2, reference_frame, rgba=rgba)                        
                    self.add_persistent_marker(marker)
                elif none_count == 2:
                    bbox = [-1, -1, 1, 1]
                    for index, x in enumerate(xyz):
                        if x is not None:
                            break
                    else:
                        index = -1
                        continue
                    if index == 0:
                        plane = 'yz'
                    elif index == 1:
                        plane = 'xz'
                    else:
                        plane = 'xy'
                    marker = rviz_tools.create_axisplane_marker(f'{config_key}.{key}', id, bbox, xyz[index], reference_frame, plane, rgba=rgba)
                    self.add_persistent_marker(marker)

    def display_object(self, config_key:str, rgba=[1.0, 0.0, 0.0, 0.8]) -> None:
        """ display one object associated with the config key

        :param config_key: the name of the config key in the scene configuration, which is either the key itself or a parent of keys
        :type config_key: str
        :param rgba: the colour of the objects defaults to [1.0, 0.0, 1.0, 0.2]
        :type rgba: list, optional
        """
        value = self.the_scene.query_config(config_key, None)
        if value is None:
            logger.warning(f'SceneToRViz (display_objects): Non-existent key {config_key}')
            return  
        subscene = self.the_scene.get_subscene_of_config_name(config_key)
        reference_frame = subscene if subscene is not None else self.base_frame              
        if type(value) != dict:
            logger.warning(f'SceneToRViz (display_objects): the key {config_key} is not associated with a single object in a dict structure')
            return
        the_object = self.the_scene._create_object_config(value)
        if the_object.type == 'box':
            xyzrpy = the_object.xyz + the_object.rpy
            marker = rviz_tools.create_cube_marker_from_xyzrpy(f'object.{config_key}', 0, xyzrpy, reference_frame, the_object.dimensions, rgba=rgba)
            self.add_persistent_marker(marker)
        elif the_object.type == 'sphere':
            marker = rviz_tools.create_sphere_marker(f'object.{config_key}', 0, the_object.xyz, reference_frame, the_object.dimensions, rgba=rgba)
            self.add_persistent_marker(marker)  
        elif the_object.type == 'object': 
            try:
                model_file = PackageFile.resolve_to_file_or_http_uri(the_object.model_file)
            except Exception as ex:
                logger.warning(f'SceneToRViz (display_objects): Invalid model_file for object ({the_object.model_file}): {ex}')
                return
            xyzrpy = the_object.xyz + the_object.rpy        
            marker = rviz_tools.create_mesh_marker(f'object.{config_key}', 0, model_file, xyzrpy, reference_frame, the_object.dimensions, rgba=rgba)
            self.add_persistent_marker(marker) 
        else:
            logger.warning(f'SceneToRViz (display_objects): Invalid object type {the_object.type} of the key {config_key}')

    # ------------------------------------------------------------------------
    # Functions for adding custom transforms for visualization
    # This class does not validate if the transform is published elsewhere
    
    def add_custom_transform(self, name, xyz, rpy, frame) -> None:
        """ Add a custom transform to the rviz visualizer, which is broadcast regularly

        :param name: the name of the transform
        :param xyz: the xyz pose
        :param rpy: the rpy pose
        :param frame: the reference frame
        """
        if name is None or xyz is None or rpy is None or frame is None:
            logger.error(f'SceneToRViz (add_custom_transform): No parameter can be None')
            raise AssertionError('A parameter is none')
        self.custom_transform_object_dict[name] = ObjectConfig(name, None, None, None, xyz, rpy, frame)

    def update_custom_transform_pose(self, name, xyz=None, rpy=None, frame=None) -> None:
        """ Update the pose of a custom transform

        :param name: the name of the transform to be updated
        :param xyz: the updated xyz, defaults to None meaning unchanged
        :param rpy: the updated rpy, defaults to None meaning unchanged
        :param frame: the updated frame, defaults to None meaning unchanged
        """
        the_object:ObjectConfig = self.custom_transform_object_dict[name]
        if the_object is None:
            logger.warning(f'__name__ (update_custom_transform_pose): the custom transform name {name} is not found')
            return False
        xyz = xyz if xyz is not None else the_object.xyz
        rpy = rpy if rpy is not None else the_object.rpy
        frame = frame if xyz is not None else the_object.frame
        self.add_custom_transform(name, xyz, rpy, frame)

    def remove_custom_transform(self, name=None):
        """ Clear a custom transform or all if name is None

        :param name: the name of the transform to be removed, or None to remove all
        """
        if name is None:
            self.custom_transform_object_dict.clear()
        elif name in self.custom_transform_object_dict:
            del self.custom_transform_object_dict[name]
    
# -----------------------------------------------------------
# test program
if __name__ == '__main__':
    the_scene = Scene(os.path.join(os.path.dirname(__file__), '../demos/gridscan/task_scene.yaml'))
    scene_to_rviz = SceneToRViz(the_scene, 'world')
    scene_to_rviz.set_publish_objects_transform(None)
    scene_to_rviz.display_bbox_regions('regions')
    scene_to_rviz.display_bbox_regions('tank.bbox', rgba=[0.2, 0.8, 0.4, 0.2])
    scene_to_rviz.display_rotations('tank.rotations', arrow_length=0.5, rgba=[0.2, 0.0, 1.0, 0.8])
    scene_to_rviz.display_positions('tank.positions', rgba=[1.0, 1.0, 0.0, 0.8])    
    # test display objects
    scene_to_rviz.display_object('objects.tank', rgba=[1.0, 0.2, 0.2, 0.8]) 
    rospy.spin()