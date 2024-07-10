# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

from collections import namedtuple, defaultdict
from task_trees.task_scene import Scene

# namedtuples for internal data structure for higher readability
TilePoseInfo = namedtuple('TilePoseInfo', ['x', 'y', 'origin_position', 'rotation'])  # representing a pose in the tiel  
TankConfig = namedtuple('TankConfig', ['object_name', 'model_file', 'scale', 'xyz', 'rpy', 'id']) # representing a tank

class GridScanScene(Scene):
    """ this class is a project specific extension of the Scene Model
        it includes the query of project specific logical poses as defined in an extension of the basic task scene configuration file 
    """
    def __init__(self, scene_config_file:str = None):
        """ the constructor
        :param scene_config_file: the path of the configuration file, refer to :py:func:`robotarchi.task_scene.SceneModel.__init__`
        :type scene_config_file: _type_, str
        """
        super(GridScanScene, self).__init__(scene_config_file)
        # -- prepare internal data structures for efficiency
        # tank configuration
        self.tank_config = self.scenes_branch['tank']
        # tile_step size
        self.step_size_x = self.tank_config['tile']['step_size_x']
        self.step_size_y = self.tank_config['tile']['step_size_y']
        # logical cartesian poses over the tiles      
        self.tileinfo = defaultdict(lambda: None)
        for tile in self.tank_config['tiles']:
            x, y = tile['tile_x'], tile['tile_y']
            origin_position = tile['origin_position']
            rotation = tile['ee_rotation']
            self.tileinfo[x, y] = TilePoseInfo(x, y, origin_position, rotation)
        # logical grid positions
        self.grid_positions_map = {}
        for pose in self.tank_config['grid_positions']:
            self.grid_positions_map[f'tank.grid_positions.{pose}'] = self.tank_config['grid_positions'][pose]
    
    # returns the tank configuration branch of the config
    def get_tank_config(self) -> dict:
        """ the tank configuration is part of the application specific scene, which models with position, orientation, scale and meshes
        :return: key-value pairs of the tank configuration as described in the config file
        :rtype: dict
        """
        return TankConfig(self.tank_config['object_name'], self.tank_config['model_file'], self.tank_config['scale'], 
                                            self.tank_pose['xyz'], self.tank_pose['rpy'], self.tank_config['id'])    
    
        # returns the named_position as a list
    def query_position_as_xyz(self, position:str, default=None) -> list:
        """ returns the named_position as a list
        :param named_position: the query named_position
        :type named_position: str
        :return: returns the position as a list
        :rtype: list
        """
        # test if position is a string and a defined string logical grid position
        if type(position) == str:            
            if position in self.grid_positions_map:
                grid_position = self.grid_positions_map[position]
                xyz = self.compute_xyz_from_grid_position(grid_position)
                return xyz
            else:
                return self.query_config(position, default)
        # test if position is a list of all numbers
        if type(position) in [list, tuple] and all([(isinstance(n, int) or n is None) for n in position]):
            if len(position) == 4:
                return self.compute_xyz_from_grid_position(position)        
            elif len(position) == 3:
                return position
        return default
    
    def compute_xyzrpy_from_grid_position(self, grid_position) -> list:
        """ maps a tile logical pose into physical pose in the format (xyzrpy)
        :param grid_position: the logical pose either as a list of (tile_x, tile_y, cell_x, cell_y) index or an alias (string)
        :type grid_position:  a list of 4 integers or a str
        :return: the position as a list in xyzrpy format
        :rtype: list
        """    
        if grid_position is None:
            return None
        # if logical_pose is an alias (str), maps to the 4 integer format according to that in the config file
        if isinstance (grid_position, str) and grid_position in self.grid_positions_map:
            grid_position = self.grid_positions_map[grid_position]
        if not (isinstance(grid_position, list) and len(grid_position) == 4):
            print(f'compute_xyzrpy_from_grid_position: parameter (grid_position) must be be a list of 4 integers or a logical position linked to the same -> fix the missing value at the function call')
            raise AssertionError(f'A parameter is invalid')  

        # first, obtain the info of (tile_x, tile_y)
        tileinfo = self.tileinfo[grid_position[0], grid_position[1]]
        if tileinfo is None:
            return (None, None, None)
        # compute the xy of the output using tile info and the indices
        xyz = [tileinfo.origin_position[0] + self.step_size_x[0] * grid_position[2] + self.step_size_y[0] * grid_position[3],
               tileinfo.origin_position[1] + self.step_size_x[1] * grid_position[2] + self.step_size_y[1] * grid_position[3], None]
        # compute the rpy of the output using tile info and the indices
        rpy = self.query_config(tileinfo.rotation)
        if rpy is None:
            rpy = self.query_config(f'tank.{tileinfo.rotation}')
            if rpy is None:
                rpy = [0, 0, 0]
        return xyz + rpy
    
    # compute the physical pose of a logical pose
    def compute_xyz_from_grid_position(self, grid_position) -> list:
        """ maps a tile logical pose into physical pose in the format (xyz)
        :param grid_position: the logical pose either as a list of (tile_x, tile_y, cell_x, cell_y) index or an alias (string)
        :type grid_position:  a list of 4 integers or a str
        :return: the position as a list in xyz format
        :rtype: list
        """
        xyzrpy = self.compute_xyzrpy_from_grid_position(grid_position)
        if xyzrpy is None:
            return None
        return xyzrpy[:3]

    # compute the physical pose of a logical pose
    def compute_rpy_from_grid_position(self, grid_position) -> list:
        """ maps a tile logical pose into physical pose in the format (rpy)
        :param grid_position: the logical pose either as a list of (tile_x, tile_y, cell_x, cell_y) index or an alias (string)
        :type grid_position:  a list of 4 integers or a str
        :return: the rotation as a list in rpy format
        :rtype: list
        """
        xyzrpy = self.compute_xyzrpy_from_grid_position(grid_position)
        if xyzrpy is None:
            return None
        return xyzrpy[3:]
    
# -----------------------------------------------------------
# test program
if __name__ == '__main__':
    
    print(f'Testing this application specific Scene class') 
      
    the_scene = GridScanScene()
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
    
    grid_positions = ['tank.transition', 'tank.zero', 'tank.start']
    for position in grid_positions:
        print(f'position {position}: {the_scene.query_position_as_xyz(position)}')