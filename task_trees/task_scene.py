# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import yaml, os, numbers
from collections import namedtuple, defaultdict
from task_trees.tools.logging_tools import logger

# namedtuples for internal data structure for higher readability
SceneKeyParts = namedtuple('SceneKeyParts', ['scene', 'parts'])
LinkConfig = namedtuple('LinkConfig', ['name', 'type', 'model_file', 'dimensions', 'xyz', 'rpy', 'parent_frame'])   # representing the scene
FrameConfig = namedtuple('FrameConfig', ['name', 'type', 'xyz', 'rpy', 'parent_frame'])

class Scene():
    """ The base model provides mapping between physical/joint-space pose and various named/logical poses.
    """
    def __init__(self, scene_config_file:str = None):
        """the constructor
        :param scene_config_file: the path to the task scene configuration file, defaults to None, which then the ros parameter
        'scene_config_file' will be used as the path if exists, and the task_scene.yaml file in the config folder is the last resort
        :type scene_config_file: str, optional
        """
        # load data from the config yaml file
        if scene_config_file is None:
            scene_config_file = os.path.join(os.path.dirname(__file__), '../demos/gridscan/task_scene.yaml')
        with open(scene_config_file, 'r') as f:
            self.config_file_root = yaml.safe_load(f)
        # print warning message
        self._print_warning()
        # valiate that there is a key 'scenes'
        if 'scenes' not in self.config_file_root:
            logger.error(f'The config file should contain a "scenes" key at the top level')
            raise AssertionError(f'Invalid yaml config file: missing the key "scenes"')
        self.scenes_branch = self.config_file_root['scenes']
        # valiate that there is a 'root' scene
        if 'root' not in self.scenes_branch:
            logger.error(f'The config file should contain a "root" scene defined under "scenes" key at the top level')
            raise AssertionError(f'Invalid yaml config file: missing the "root" scene')
        self.root_scene_config = self.scenes_branch['root']        
        # model parameters
        self.scenes_dict = defaultdict(lambda: None)
        self.scenes_linkconfig = defaultdict(lambda: None)
        self.frames_config = defaultdict(lambda: None)
        # iterates all scenes in the config file
        for key in self.scenes_branch.keys():
            self.scenes_dict[key] = self.scenes_branch[key]  
            link_config = self._extract_link_config(key, self.scenes_branch[key])
            if link_config is not None:
                if link_config.type == 'frame':
                    self.frames_config[key] = link_config
                else:
                    self.scenes_linkconfig[key] = link_config

    # internal function: parse a dot-separated config kay (path) into scene name and a list of the parts 
    def _parse_config_key(self, config_key:str) -> SceneKeyParts:
        assert config_key is not None, 'Parameter (config_key) should not be None'
        parts = config_key.split('.')
        assert len(parts) >= 1, 'Parameter (config_key) is empty'
        scene_name = parts[0]
        if scene_name not in self.scenes_dict:
            # the first name is not a known scene, assume that the root scene is inferred
            scene_name = 'root'
            return SceneKeyParts(scene_name, parts)
        else:
            return SceneKeyParts(scene_name, parts[1:])
        
    # -------------------------------------------
    # internal functions for extracting objects and frames

    def _extract_link_config(self, scene_name:str, scene_dict:dict) -> LinkConfig:
        return self._create_link_config(scene_name, scene_dict.get('link', None))
    
    # internal function: extract and validate an object definition
    def _create_link_config(self, scene_name:str, link_dict:dict) -> LinkConfig:
        if link_dict is None or scene_name is None:
            return None
        # if 'name' not in link_dict:
        #     logger.error(f'Scene (_process_an_object): the keys in the link config contains no "name"')
        #     return None
        if 'type' not in link_dict or 'xyz' not in link_dict or 'rpy' not in link_dict:
            logger.error(f'Scene (_process_an_object): the keys in the scene config ({link_dict["name"]}) contains no "type", "xyz", "rpy"')
            return None 
        if link_dict['type'] not in ['box', 'sphere', 'object', 'frame']:
            logger.error(f'Scene (_process_an_object): the type of the scene ({link_dict["name"]}) is not one of "sphere", "box", "frame", or "object"')
            return None 
        if link_dict['type'] in ['box', 'sphere', 'object'] and 'dimensions' not in link_dict:
            logger.error(f'Scene (_process_an_object): the keys in the box, sphere or object typed scene link ({link_dict["name"]}) contains no "dimensions"')
            return None    
        model_file = link_dict.get('model_file', None)
        if link_dict['type'] == 'object' and model_file is None:
            logger.error(f'Scene (_process_an_object): the scene link ({link_dict["name"]}) expects a valid "model_file" key')
            return None
        parent_name = link_dict.get('parent_frame', link_dict.get('frame', None))  # both parent_frame and frame are acceptable
        if parent_name is not None and parent_name == 'root' and 'root' not in self.scenes_linkconfig:
            parent_name = None
        dimensions = link_dict.get('dimensions', None)
        return LinkConfig(scene_name, link_dict['type'], model_file, dimensions, link_dict['xyz'], link_dict['rpy'], parent_name)

    def _print_warning(self) -> None:
        if not hasattr(self, 'warned'):
            self.warned = True
            logger.warning(f'''Scene configuration file format changed in task_trees 0.2.x. 
            The subscene concept is removed and scenes are equal in the yaml file
            scenes:
                scene_1_name:
                    link: ...
                    other_params: ...
                    ...
                scene_2_name:
                    link: ...
            The API is also changed. Please refer to the documentation''')
  
    # -------------------------------------------
    # config query functions

    # returns the value of a named config given as a dot-separated key (e.g. table.objects.lamp.position)
    def query_config(self, key:str, default=None):
        """ returns the value of any named config given as a dot-separated path
        :param key: the dot-separated key, the root scene may have the "root" omitted
        :type key: str
        :param default: the default value if nothing is found in the key, defaults to None
        :type default: any
        :return: the value of the config
        :rtype: any
        """
        the_parsed_key = self._parse_config_key(key)
        if the_parsed_key is None:
            return default
        pointer = self.scenes_dict[the_parsed_key.scene]
        for item in the_parsed_key.parts:
            if item.isnumeric():
                try:
                    item = int(item)
                except:
                    logger.error(f'query_config: part of query key {the_parsed_key} contains an non-integer index {item}')
                    raise AssertionError(f'query_config: Non-existent config name "{the_parsed_key}", which contains an invalid index {item}')
                if item < 0 or item >= len(pointer):
                    logger.error(f'query_config: part of query key {the_parsed_key} contains an out-of-range integer index {item} ')
                    raise AssertionError(f'query_config: Non-existent config name "{the_parsed_key}", which contains an invalid index {item}')
                if type(pointer) not in (tuple, list):
                    logger.error(f'query_config: part of query key {the_parsed_key} contains an invalid index {item}, expecting a string key')
                    raise AssertionError(f'query_config: Non-existent config name "{the_parsed_key}", which contains an invalid index {item}')                    
                pointer = pointer[item]
            else:
                if type(pointer) != dict:
                    logger.error(f'query_config: part of query key {the_parsed_key} contains an invalid index {item}, expecting an integer')
                    raise AssertionError(f'query_config: Non-existent config name "{the_parsed_key}", which contains an invalid index {item}')                    
                if item not in pointer:
                    logger.error(f'query_config: part of query key {the_parsed_key} contains a non-existent index {item}')
                    raise AssertionError(f'query_config: Non-existent config name "{the_parsed_key}", which contains an invalid index {item}')                    
                pointer = pointer[item]
        return pointer 
    
    # returns True of the given config name exists
    def exists_config(self, key:str) -> bool:
        """ returns True of the given config name exists
        :param key: The config name
        :type key: str
        :return: True if the given config name exists
        :rtype: bool
        """
        return self.query_config(key) != None
    
    # returns a list of keys specified under the config key if it is a dictionary or an empty list
    def key_list_under_config_key(self, key:str) -> list:
        """ returns a list of keys specified under the config name or an empty list
        :param key: The config name
        :type key: str
        :return: a list of keys specified under the config name
        :rtype: list
        """
        pointer = self.query_config(key)
        if pointer is None or type(pointer) not in [dict, map]:
            return []
        return list(pointer.keys())
    
    # returns the length of the list if the config key is a list or an empty list
    def len_of_key_list_under_config_key(self, key:str) -> int:
        """ returns the length of the list under the config name
        :param key: The config name
        :type key: str
        :return: the length of the list the config name 
        :rtype: int
        """        
        pointer = self.query_config(key)
        if pointer is None or type(pointer) not in [tuple, list]:
            return 0
        return len(pointer)     

    # list the scene names
    def list_scene_names(self) -> list:
        """ return the list of objects defined in the scene configuration
        :return: a list of object names as strings
        :rtype: list
        """
        return list(self.scenes_dict.keys())
    
    # return scene config given the scene name
    def get_scene_config_as_dict(self, scene_name: str) -> dict:
        """ return the scene configuration as a dict, or None if the scene not exists
        :param scene_name: name of the object
        :type scene_name: str
        :return: the configuration of the scene as a dict or None if not exists
        :rtype: dict
        """
        return self.scenes_dict.get(scene_name, None)  
    
    # return config given the object name
    def get_link_of_scene(self, scene_name: str) -> LinkConfig:
        """ return the object configuration as ObjectConfig tuple, or None if the name not exists
        :param scene_name: name of the scene
        :type scene_name: str
        :return: the LinkConfig of the scene or None if not exists
        :rtype: LinkConfig
        """
        return self.scenes_linkconfig.get(scene_name, None)

    # list the frame names
    def list_frame_names(self) -> list:
        """ return the list of frame type scene defined in the scene configuration
        :return: a list of frame names as strings
        :rtype: list
        """
        return list(self.frames_config.keys())    
    
    # return scene config given the scene name
    def get_frame_config_as_dict(self, frame_name: str) -> dict:
        """ return the frame configuration as a dict, or None if the scene not exists
        :param frame_name: name of the frame
        :type frame_name: str
        :return: the configuration of the frame type scene or None if not exists
        :rtype: dict
        """
        return self.scenes_dict.get(frame_name, None)  
    
    # return config given the object name
    def get_link_of_frame(self, frame_name: str) -> LinkConfig:
        """ return the object configuration as ObjectConfig tuple, or None if the name not exists
        :param frame_name: name of the frame
        :type frame_name: str
        :return: the LinkConfig of the frame or None if not exists
        :rtype: LinkConfig
        """
        return self.frames_config.get(frame_name, None)      

    # return the named poses as a dict
    def get_named_poses_of_root_as_dict(self) -> dict:
        """ return the named poses defined in the scene configuration as a dict

        :return: dict containing key value pairs of named poses
        """
        result = dict()
        named_poses = self.query_config('named_poses')
        for pose_name in named_poses:
            pose_name = 'named_poses.' + pose_name
            result[pose_name] = self.query_config(pose_name)
        return result



# -----------------------------------------------------------
# test program
if __name__ == '__main__':
    the_scene = Scene()
    named_poses = ['named_poses.stow', 'named_poses.home']
    for named_pose in named_poses:
        print(f'named_pose {named_pose}: {the_scene.query_config(named_pose)}')

    regions = ['regions.workspace', 'regions.inner', 'regions.work', 'regions.work3d']
    for region in regions:
        print(f'region {region}: {the_scene.query_config(region)}')

    positions = ['tank.positions.default', 'tank.positions.hover', 'tank.positions.submerged']
    for position in positions:
        print(f'position {position}: {the_scene.query_config(position)}')
        
    rotations = ['tank.rotations.alpha', 'tank.rotations.beta', 'tank.rotations.delta', 'tank.rotations.gamma']
    for rotation in rotations:
        print(f'rotation {rotation}: {the_scene.query_config(rotation)}')        
    
    