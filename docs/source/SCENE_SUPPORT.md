# Logical Scene Configuration Support

The separation of logical scene and physical scene is a critical feature of the task trees framework. It can largely eliminate the need to modify the source code due to changes in the physical scene, including the environment and the robot manipulator physcial attributes. 

The use of string-based logical poses (positions and rotations) also improves code readability, facilitate reusability, and eases code maintainance. 

The key enabling component of the logical scene support is **scene configuration file**, which is a yaml file comprising definitions of logical positions and rotations in the scene, definitions of subscenes and collision objects. 

## Scene Configuration File

### Basic Structure of a Scene Configuration File

A scene configuration file can be divided into one or more scenes. The first scene must be known as `root`. The scene is the top-level identifier for refering to all the resource keys in the configuration file. Each scene defines its own reference frame and optionally an visual object attached. A scene without a specified visual object is called a frame-type scene or simply known as a frame. 

The significance betwwen the root scene and the other scenes is the namespace of their configuration key-value pairs. All key-value defined under the root scene may be replaced with an empty namespace, while the variables under a subscene requires the subscene name as the namespace.  

The configuration file must starts with the key `scenes`, under which each scene can be added by the name as the head the scene branch. In the following example, `root` is the mandatory scene, and `scene_1` and `scene_2` are names of custom scenes.

```diff
---
scenes:
  root:
  ... variables of the root scene go here

    named_poses:
      ... optional definitions of named poses

  scene_1:
    link: 
      ... pose definitions of scene (which are also reference frames)
    ... variables of the subscene scene_1 go here

  scene_2:
    ... variables of the subscene scene_2 go here
```

### Specifying Configurations in the Root Scene and Other Scenes

Consider the example `task_scene_4.yaml` [source code](../../demos/simple_moves_scene/task_scene_4.yaml) below. In this example, all configuration keys are under the root scene. No subscene is defined.

```
---
scenes:
  root:
    named_poses: 
      home: [0.0, -0.785, 0.0, -2.36, 0.0, 1.57, 0.785] # from base
    positions:
      default_z: [null, null, 0.5]
  area_1:
    link:
      parent_frame: root
      type: box
      model_file: null
      dimensions: [0.2, 0.2, 0.01]
      xyz: [0.4, 0.20, 0.00]
      rpy: [0, 0, 3.14]
  area_2:
    link:
      parent_frame: root
      type: box
      model_file: null
      dimensions: [0.2, 0.2, 0.01]
      xyz: [0.4, -0.20, 0.00]
      rpy: [0, 0, 3.14]
```
A scene is firstly a namespace for queries and key references. It should also be the reference frame to which the poses are defined.  The scene name is both the name of a reference frame and, if defined, a collision object. 

The scene configuration file below defines one scene called `area_1`, in which several configuration keys are defined. Note that `area_1` is a reference frame (a side effect of defining a collision object). Normally, pose keys such as `area_1.positions.start` and their scene (referene frame) `area_1` are used together in using the move behaviours.
```
scenes:
  area_1:
    link:
      parent_frame: root
      type: box
      model_file: null
      dimensions: [0.2, 0.2, 0.01]
      xyz: [0.4, 0.20, 0.00]
      rpy: [0, 0, 3.14]
    positions:
      hover: [null, null, 0.40]
      down: [null, null, 0.27]
      start: [0, 0.11, null]
      centre: [0, -0.015, null] # -0.01
      restart: [0, -0.185, null]
      end: [0, -0.06, null]
```

### The `link` Branch: Collision Objects and Custom Reference Frames 

The link of a scene specifies a reference frame and optionally the collision object.

Collision object definitions comprise the following mandatory and optional keys headed by the name of the object.
- `type`: `box`, `sphere`, or `object`
- `model_file`: the path to the mesh file if the type is `object`, which supports full file path, package relative file path, the `file`, `package` and `http` uri.
- `dimensions`: a list of length, width, and height for a `box`, the radius for a `sphere`, and the scaling factor as a list of 3 numbers for an `object`.
- `xyz`: the position of the object
- `rpy`: the orientation of the object  
- `parent_frame`: the reference frame for the object pose, where `null` implies the world frame. This field is optional.

Every collision object will project its position and rotation as its own custom reference frames. The following defines the scene named `area_1` which is also the name of the reference frame. In particular, the type `box` indicaates that it is a collision object as well. Note that the null frame is default to the base/world frame. The pose parameter
 (xyz and rpy) is the transform from `area_1` to the base/world frame. 
```
  area_1:
    link:
      type: box
      model_file: null
      dimensions: [0.2, 0.2, 0.01]
      xyz: [0.4, 0.20, 0.00]
      rpy: [0, 0, 3.14]
      frame: null
```
Collision objects can be defined based on the reference of another scene. If `parent_frame` is not given or its value is `null`, the scene becomes the default frame.

Custom reference frame definitions comprise the following mandatory and optional keys headed by the name of the frame.
- `type`: `frame`
- `xyz`: the position of the transform
- `rpy`: the orientation of the transform 
- `parent_frame` or `frame`: the reference frame for the custom frame, where `null` implies the world frame. This field is optional.

Similarly, custom reference frames can be defined with another scene as the parent frame. The default frame is the root scene.

### Loading and Querying the Scene Configuration File

To load the query the logical scene configuration, the task trees framework offers the class `task_trees_task_scene.Scene` as an alternative. The class recognizes definitions of scene and subscenes and provides a more convenient way to query and obtain configuration values.

```
from task_trees.task_scene import Scene
...
    self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene_4.yaml'))
```
The `Scene` class gives every configuration key a unique reference. The reference is a dot-separated string made up of the scene namespace followed by keys of the ancestors of a key-value configuration. For example, the reference `named_poses.home` refers to the `home` child of the `named_poses` node under the root scene (therefore the empty namespace). A configuration key under a subscene has a prepended namespace. For example, `area.positions.start` refers to the value `[0, 0.11, null]` in the above example.

The following code snippet demonstrates the functions `key_list_under_config_key` and `query_config` for listing the children keys and querying for the value of a configuration reference.

```
        self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene.yaml'))

        # setup name poses
        self.named_poses = self.the_scene.key_list_under_config_key('named_poses')
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
### Branches Recognized by the Task Tree Manager

The named branches of `link`, and `named_poses` have dedicated functions in the scene configuration.

- The key `link` under a scene with the `type` equals to `box`, `sphere`, and `object` is recognized by the task tree manager `TaskTreesManager` class as definitions of collision objects. In the task trees, collision objects are also recognized as custom reference frames. 
- The type of `frames` under the key `link` of a scene is recognized as definitions of custom reference frames. 
- The key`named_poses` under the root scene is also recognized by the task tree manager as the named poses adding to the arm commander.

The `TaskTreesManager` class offers the functions `_define_links`, `_define_frames` and `_define_named_poses` for adding the scenes/collision objects, the custom reference frames, and named poses defined in the scene configuration.


### Internal Logical References

Subclasses of `SceneConditionalCommanderBehaviour` accepts internal references when resolving logical positions and rotations. This is handy in avoiding redundency in scene configuration.

The following part of a configuration file is extracted from `task_scene.yaml` [source code](../../demos//pushblock/task_scene.yaml) under `/demos/pushblock`.
```
scenes:
  root:
    named_poses: 
      stow: [0.0, -1.244, 0.0, -2.949, 0.0, 1.704, 0.785] # from base
      home: [0.0, -0.785, 0.0, -2.36, 0.0, 1.57, 0.785] # from base
    regions:
      workspace: [-0.5, -0.75, -0.1, 1.0, 0.5, 1.0]  # min_x, min_y, min_z, max_x, max_y, max_z
      inner: [-0.5, -0.75, 0.30, 0.5]  # min_x, min_y, max_x, max_y
    rotations:
      # alpha: [3.14, null, 0.78] 
      # beta: [3.14, null, -0.78]
      alpha: [3.14, 0, -0.785] 
      beta: [3.14, 0, 2.358]      
    push_block:
        dimensions: [0.05, 0.05, 0.05]
        xyz: [0.0, 0.025, 0.151]
        rpy: [0, 0, 0]    
...
  area_1:
    rotation: rotations.alpha
    link:
      type: box
      model_file: null
      dimensions: [0.1, 0.1, 0.25]
      xyz: [0.4, 0.0, 0.125]
      rpy: [0, 0, 1.57]
  area_2:
    rotation: rotations.beta
    link:
      type: box
      model_file: null
      dimensions: [0.1, 0.1, 0.25]
      xyz: [0.50, -0.10, 0.125]
      rpy: [0, 0, 3.14]
  area_3:
    rotation: rotations.beta
    link:
      type: box
      model_file: null
      dimensions: [0.1, 0.1, 0.25]
      xyz: [0.6, 0.0, 0.125]
      rpy: [0, 0, -1.57]
  area_4:
    rotation: rotations.alpha
    link:
      type: box
      model_file: null
      dimensions: [0.1, 0.1, 0.25]
      xyz: [0.50, 0.10, 0.125]
      rpy: [0, 0, 0] 
```
The key `rotation` of the four area scenes are mapped to either `rotation.alpha` or `rotation.beta`. For example, in resolving the key `area_1.rotation`, if the value is a string, `SceneConditionalCommanderBehaviour` attempts to resolve it once more using the scene configuration. 

The internal logical reference feature improves the readability and maintainability of the configuration file.

## The Scene class

The functions provided by the `Scene` class are listed below.

| Functions | Remarks | 
| --- | --- |
| `query_config(self, name:str, default=None)` | The `name` is in the format of dot separated reference |  
| `exists_config(self, name:str)` | Return True if the configuration name exists |
| `key_list_under_config_key(self, key:str)` | Return the children keys of a configuration reference as a list |
| `len_of_key_list_under_config_key(self, key:str)` | Return the number of children under a list-type configuration reference |
| `list_scene_names(self)` | Return the names of the defined scenes as a list |
| `get_scene_config_as_dict(self, scene_name)` | Return the scene config branch in the config file as a dict |
| `get_link_of_scene(self, scene_name)`| Return the link of the scene as a LinkConfig |
| `list_frame_names(self)` | Return the names of the defined frame-type scenes as a list |
| `get_frame_config_as_dict(self, frame_name)` | Return the frame-type scene config branch in the config file as a dict |
| `get_link_of_frame(self, frame_name)`| Return the link of the frame as a LinkConfig |
| `get_named_poses_of_root_as_dict(self, name)`| Return the config of a subscene as a dict |   

Refer to the API Reference for more details about these functions.

### Subclassing the Scene class

Any configuration key-value pair can be queried using the `Scene` class. A subclass of `Scene` can be developed to add computation to the configurations.

For example, in the `gridscan` demo application [source code](../../demos/gridscan/demo.py), the physical position in a grid is determined by the grid cell index and the size of grid cell. The class `GridScanScene` was developed to compute the conversion of grid cell index to the corresponding physical position, based on the grid cell size specified in the scene configuration file.

## Customized Configuration File

The function `query_config` allows the query of any configuration key using a dot-separated query key. Each item in the query key corresponds to a node in the configuration yaml file. The item to reference a list-list structure must be a zero-starting integer. The item to reference a dict-like structure must be a string. 

For example, tke query key `positions.start` refers to a configuration key under the root scene.
```
scenes:
  root:
    positions:
      start: [1, 2, 3]
      end: [2, 3, 5]
```
On the other hand, the query key `grid.1.rotation.alpha` matches the following structure, and it refers to the value [0, 0, 1.57]. The `1` refers to the index `1` of the list-like child under `grid`.
```
scenes:
  root:
    grid:
      - rotation:
          alpha: [3.14, 0, 0]
      - rotation:
          alpha: [0, 0, 1.57]
```

To refer to a configuration key under a non-root scene, the query key should start with the scene name. Given the following configuration yaml file.
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

