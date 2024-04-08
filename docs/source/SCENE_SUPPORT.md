# Logical Scene Configuration Support

The separation of logical scene and physical scene is a critical feature of the task trees framework. It can largely eliminate the need to modify the source code due to changes in the physical scene, including the environment and the robot manipulator physcial attributes. 

The use of string-based logical poses (positions and rotations) also improves code readability, facilitate reusability, and eases code maintainance. 

The key enabling component of the logical scene support is **scene configuration file**, which is a yaml file comprising definitions of logical positions and rotations in the scene, definitions of subscenes and collision objects. 

## Scene Configuration File

### Basic Structure of a Scene Configuration File

A scene configuration file is divided into the root scene and custom subscene. The significance betwwen the root scene and the subscenes is the namespace of their configuration key-value pairs. All key-value defined under the root scene do have an empty namespace, while the variables under a subscene requires the subscene name as the namespace.  The top level keys must be `scene` and `subscenes`, and under `subscenes` keys representing names of subscenes can be added. In the following example, `sub_scene_1` and `sub_scene_2` are names of custom subscenes.

```diff
---
scene:
  ... variables of the root scene go here
    objects: 
      ... optional definitions of objects and reference frames
    named_poses:
      ... optional definitions of named poses

subscenes:
    sub_scene_1:
      ... variables of the subscene sub_scene_1 go here

    sub_scene_2:
      ... variables of the subscene sub_scene_2 go here
```

### Optional Branches Recognized by the Task Tree Manager

The key `objects` under the root scene is recognized by the task tree manager `TaskTreesManager` class as definitions of collision objects and custom reference frames. In the task trees, collision objects are also recognized as custom reference frames. 

Similarly, the key `named_poses` under the root scene is also recognized by the task tree manager as the named poses adding to the arm commander.

Both of these branches are optional.

The `TaskTreesManager` class offers the functions `_define_objects` and `_define_named_poses` for defining the objects and named poses according to the scene configuration.


### Specifying Configurations in the Root Scene and Subscenes

Consider the example `task_scene_4.yaml` [source code](../../demos/simple_moves_scene/task_scene_4.yaml) below. In this example, all configuration keys are under the root scene. No subscene is defined.

```
---
scene:
  named_poses: 
    home: [0.0, -0.785, 0.0, -2.36, 0.0, 1.57, 0.785] # from base
  positions:
    default_z: [null, null, 0.5]
  objects:
    area_1:
      type: box
      model_file: null
      dimensions: [0.2, 0.2, 0.01]
      xyz: [0.4, 0.20, 0.00]
      rpy: [0, 0, 3.14]
      frame: null
    area_2:
      type: box
      model_file: null
      dimensions: [0.2, 0.2, 0.01]
      xyz: [0.4, -0.20, 0.00]
      rpy: [0, 0, 3.14]
      frame: area_1
subscenes:
```
The scene configuration file below defines one subscene called 'area', in which several configuration keys are defined.
```
subscenes:
  area:
    positions:
      hover: [null, null, 0.40]
      down: [null, null, 0.27]
      start: [0, 0.11, null]
      centre: [0, -0.015, null] # -0.01
      restart: [0, -0.185, null]
      end: [0, -0.06, null]
```

### Collision Objects and Custom Reference Frames

The collision object definition comprises the following mandatory keys headed by the name of the object.
- `type`: `box`, `sphere`, or `object`
- `model_file`: the path to the mesh file if the type is `object`
- `dimensions`: a list of length, width, and height for a `box`, the radius for a `sphere`, and the scaling factor as a list of 3 numbers for an `object`.
- `xyz`: the position of the object
- `rpy`: the orientation of the object  
- `frame`: the reference frame against which the object pose is measured, where null implies the world frame. This field is optional.

Every collision object will project its position and rotation as its own custom reference frames. The following defines the object named `area_1` and the fields are populated. Note that the null frame is default to the world frame.
```
    area_1:
      type: box
      model_file: null
      dimensions: [0.2, 0.2, 0.01]
      xyz: [0.4, 0.20, 0.00]
      rpy: [0, 0, 3.14]
      frame: null
```
A collision object can be defined under a subscene as well. The default frame is that subscene.

### Loading and Querying the Scene Configuration File

To load the query the logical scene configuration, the task trees framework offers the class `task_trees_task_scene.Scene` as an alternative. The class recognizes definitions of scene and subscenes and provides a more convenient way to query and obtain configuration values.

```
from task_trees.task_scene import Scene
...
    self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene_4.yaml'))
```
The `Scene` class gives every configuration key a unique reference. The reference is a dot-separated string made up of the scene namespace followed by keys of the ancestors of a key-value configuration. For example, the reference `named_poses.home` refers to the `home` child of the `named_poses` node under the root scene (therefore the empty namespace). A configuration key under a subscene has a prepended namespace. For example, `area.positions.start` refers to the value `[0, 0.11, null]` in the above example.

The following code snippet demonstrates the functions `keys_of_config` and `query_config` for listing the children keys and querying for the value of a configuration reference.

```
        self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene.yaml'))

        # setup name poses
        self.named_poses = self.the_scene.keys_of_config('named_poses')
        for pose_name in self.named_poses:
            pose_name = 'named_poses.' + pose_name
            self.arm_commander.add_named_pose(pose_name, self.the_scene.query_config(pose_name))
```

### Partial Specification of Positions and Rotations

The `null` value signifies that the component is to be resolved by other specification in a compositional move target. For example the configuration `positions.default_z` has the value `[null, null, 0.5]`. Only the z component is specified, and the other components are omitted.

In the following compositional move target example, the missing x and y components are to be provided by the function `self.generate_random_xyz`.
```
        DoMoveXYZ('move_xy', True, arm_commander=self.arm_commander, scene=self.the_scene, 
            target_xyz=['positions.default_z', self.generate_random_xyz], reference_frame='area_2'),   
```

### Internal Logical References

Subclasses of `SceneConditionalCommanderBehaviour` accepts internal references when resolving logical positions and rotations. This is handy in avoiding redundency in scene configuration.

The following part of a configuration file is extracted from `task_scene.yaml` [source code](../../demos//pushblock/task_scene.yaml) under `/demos/pushblock`.
```
scene:
  named_poses: 
    stow: [0.0, -1.244, 0.0, -2.949, 0.0, 1.704, 0.785] # from base
    home: [0.0, -0.785, 0.0, -2.36, 0.0, 1.57, 0.785] # from base
  regions:
    workspace: [-0.5, -0.75, -0.1, 1.0, 0.5, 1.0]  # min_x, min_y, min_z, max_x, max_y, max_z
    inner: [-0.5, -0.75, 0.30, 0.5]  # min_x, min_y, max_x, max_y
  positions:
    drop: [0.5, 0.3, 0.4]
  rotations:
    alpha: [3.14, 0, -0.785] 
    beta: [3.14, 0, 2.358]      
...
subscenes:
  area_1:
    rotation: rotations.alpha
  area_2:
    rotation: rotations.beta
  area_3:
    rotation: rotations.beta
  area_4:
    rotation: rotations.alpha
```
The key `rotation` of the four subscenes are mapped to either `rotation.alpha` or `rotation.beta`. For example, in resolving the key `area_1.rotation`, if the value is a string, `SceneConditionalCommanderBehaviour` attempts to resolve it once more using the scene configuration. 

The internal logical reference feature improves the readability and maintainability of the configuration file.

## The Scene class

The functions provided by the `Scene` class are listed below.

| Functions | Remarks | 
| --- | --- |
| `query_config(self, name:str, default=None)` | The `name` is in the format of dot separated reference |  
| `query_position_as_xyz(self, name, default=None)` | Alias of `query_config` for backward compatability |
| `query_rotation_as_rpy(self, name, default=None)` | Alias of `query_config` for backward compatability |
| `get_scene_config(self)` | Return the root scene config in the config file as a dict |
| `get_subscene_config(self, name)`| Return the config of a subscene as a dict |   
| `exists_config(self, name:str)` | Return True if the configuration name exists |
| `keys_of_config(self, name:str)` | Return the children of a configuration reference as a list of keys |  
| `len_of_list_config(self, name:str)` | Return the number of children of a list typed configuration reference |
| `list_object_names(self)` | Return the names of the defined objects as a list |
| `get_object_config(self, object_name: str)` | Return the configuration of an object given its name |

Refer to the API Reference for more details about these functions.

### Subclassing the Scene class

Any configuration key-value pair can be queried using the `Scene` class. A subclass of `Scene` can be developed to add computation to the configurations.

For example, in the `gridscan` demo application [source code](../../demos/gridscan/demo.py), the physical position in a grid is determined by the grid cell index and the size of grid cell. The class `GridScanScene` was developed to compute the conversion of grid cell index to the corresponding physical position, based on the grid cell size specified in the scene configuration file.

## Customized Configuration File

The function `query_config` allows the query of any configuration key using a dot-separated query key. Each item in the query key corresponds to a node in the configuration yaml file. The item to reference a list-list structure must be a zero-starting integer. The item to reference a dict-like structure must be a string. 

For example, tke query key `positions.start` refers to a configuration key under the root scene.
```
scene:
  positions:
    start: [1, 2, 3]
    end: [2, 3, 5]
```
On the other hand, the query key `grid.1.rotation.alpha` matches the following structure, and it refers to the value [0, 0, 1.57]. The `1` refers to the index `1` of the list-like child under `grid`.
```
scene:
  grid:
    - rotation:
        alpha: [3.14, 0, 0]
    - rotation:
        alpha: [0, 0, 1.57]
```

To refer to a configuration key under a subscene, the query key should start with the subscene name. Given the following configuration yaml file.
```
subscenes:
  the_tank:
    grid
      - rotation:
          alpha: [3.14, 0, 0]
      - rotation:
          alpha: [0, 0, 1.57]
```
The query key `the_tank.grid.0.rotation.alpha` refers to the value `[3.14, 0, 0]`.

Application developers can design a suitable configuration structure and use the function `query_config` to rerieve configuration values.



### Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Mar 2024

