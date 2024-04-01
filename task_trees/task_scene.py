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
      
# namedtuples for internal data structure for higher readability
ObjectConfig = namedtuple('ObjectConfig', ['name', 'type', 'model_file', 'dimensions', 'xyz', 'rpy']) # representing an object
SceneName = namedtuple('SceneName', ['subscene', 'path'])

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
            scene_config_file = os.path.join(os.path.dirname(__file__), 'task_scene.yaml')
        with open(scene_config_file, 'r') as f:
            config = yaml.safe_load(f)
        self.config = config
        # if the default scene is defined in the file, define a variable for the default
        self.scene_config = self.config['scene'] if 'scene' in self.config else None
        # if subscenes are defined in the file, define another variable for that
        self.subscene_config = self.config['subscenes'] if 'subscenes' in self.config else None

        # preprocess the objects
        self.objects_map = self._process_objects()

    # internal function: parse a dot-separated config name (path) into two parts, the subscene and a list of the path
    def _parse_config_name(self, config_name:str) -> SceneName:
        if config_name is None:
            return None
        parts = config_name.split('.')
        if self.subscene_config is not None and len(parts) > 1 and (parts[0] in self.subscene_config): # one of the subscenes
            return SceneName(parts[0], parts[1:])
        else:
            return SceneName(None, parts)

    # internal function: returns a map of (name, value) of objects defined in the config file
    def _process_objects(self) -> dict:
        the_map = dict()
        # process the main scene
        if self.scene_config is not None and 'objects' in self.scene_config:
            for obj in self.scene_config['objects']:
                the_map[obj['name']] = ObjectConfig(obj['name'], obj['type'], obj['model_file'], obj['dimensions'], obj['xyz'], obj['rpy'])  
        # process each subscene
        if self.subscene_config is not None:
            for subscene in self.subscene_config.keys():
                if 'objects' in self.subscene_config[subscene]:
                    for obj in self.subscene_config[subscene]['objects']:
                        the_map[f'{subscene}.{obj["name"]}'] = ObjectConfig(obj['name'], obj['type'], obj['model_file'], obj['dimensions'], obj['xyz'], obj['rpy'])  
        return the_map    

    # -------------------------------------------
    # model functions

    # returns the value of a named config given as a dot-separated path (e.g. table.objects.lamp.position)
    def query_config(self, name:str, default=None):
        """ returns the value of any named config given as a dot-separated path
        :param name: the dot-separated path
        :type name: str
        :param default: the default value if nothing is found in the path, defaults to None
        :type default: any
        :return: the value of the config
        :rtype: any
        """
        scene_name = self._parse_config_name(name)
        if scene_name is None:
            return default
        if scene_name.subscene is not None:
            pointer = self.subscene_config[scene_name.subscene]
        else:
            pointer = self.scene_config

        for item in scene_name.path:
            if item.isnumeric():
                item = int(item)
                if item < 0 or item >= len(pointer):
                    print(f'query_config: part of the name contains an invalid index {item}')
                    raise AssertionError(f'query_config: Non-existent config name "{name}", which contains an invalid index {item}')
                pointer = pointer[item]
            else:
                if item not in pointer:
                    print(f'query_config: part of the name contains an invalid key {item}')
                    raise AssertionError(f'query_config: Non-existent the config name "{name}", which contains an invalid key {item}')
                pointer = pointer[item]
        return pointer
    
    # alias for backward compatibility query_config 
    def query_position_as_xyz(self, name, default=None):
        return self.query_config(name, default)
    
    def query_rotation_as_rpy(self, name, default=None):
        return self.query_config(name, default)    

    # returns the root scene config as a dict
    def get_scene_config(self) -> dict:
        """ returns the main scene config in the config file as a dict
        :return: the content in the config file
        :rtype: dict
        """
        return self.scene_config
    
    # returns a subscene config as a dict
    def get_subscene_config(self, name) -> dict:
        """ returns the config of a subscene in the config file as a dict
        :return: the content in the config file about the subscene or None
        :rtype: dict
        """
        if name not in self.subscene_config:
            return None
        return self.subscene_config[name]
    
    # returns True of the given config name exists
    def exists_config(self, name:str) -> bool:
        """ returns True of the given config name exists
        :param name: The config name
        :type name: str
        :return: True if the given config name exists
        :rtype: bool
        """
        return self.query_config(name) != None
    
    # returns a list of keys specified under the config name
    def keys_config(self, name:str) -> list:
        """ returns a list of keys specified under the config name or an empty list
        :param name: The config name
        :type name: str
        :return: a list of keys specified under the config name
        :rtype: list
        """
        pointer = self.query_config(name)
        if pointer is None or type(pointer) not in [dict, map]:
            return []
        return list(pointer.keys())
    
    # returns the length of the list under the config name or an empty list
    def len_config(self, name:str) -> int:
        """ returns the length of the list under the config name
        :param name: The config name
        :type name: str
        :return: the length of the list the config name 
        :rtype: int
        """        
        pointer = self.query_config(name)
        if pointer is None or type(pointer) not in [tuple, list]:
            return 0
        return len(pointer)     

    # list the object names
    def list_object_names(self) -> list:
        """ 
        :return: a list of object names as strings
        :rtype: list
        """
        if self.objects_map is None:
            return []
        return self.objects_map.keys()
    
    # return config given the object name
    def get_object_config(self, object_name: str) -> ObjectConfig:
        """ return the object configuration as ObjectConfig tuple, or None if the name not exists
        :param object_name: name of the object
        :type object_name: str
        :return: the configuration as a tuple or None if not exists
        :rtype: ObjectConfig
        """
        return self.objects_map.get(object_name)

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
    
    objects = the_scene.list_object_names()
    for object_name in objects:
        print(f'object {object_name}: {the_scene.get_object_config(object_name)}')
    
    print(f'tank tile step_sizes: {the_scene.query_config("tank.tile")}')
    