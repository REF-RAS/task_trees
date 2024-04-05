# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

# Reference: https://raw.githubusercontent.com/DavidB-CMU/rviz_tools_py/master/src/rviz_tools_py/rviz_tools.py

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import yaml, os, time, numbers, threading, random
from enum import Enum
from collections import defaultdict
import rospy, tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3, Point
from visualization_msgs.msg import Marker
from arm_commander.moveit_tools import create_pose
from task_trees.tools import logger

class RGBAColors(int, Enum):
    RED = 0, (1.0, 0.0, 0.0, 0.5)
    BLUE = 1, (0.0, 0.0, 1.0, 0.5)
    GREEN = 2, (0.0, 1.0, 0.0, 0.5)
    def __new__(cls, value, rgba='...'):
        obj = int.__new__(cls, value)
        obj._value_ = value
        obj.rgba = rgba
        return obj
    
def create_marker(name:str, id:int, reference_frame:str=None, lifetime=rospy.Duration(0.0)):
    the_marker = Marker()
    the_marker.header.frame_id = reference_frame
    the_marker.header.stamp = rospy.Time()
    the_marker.action = Marker.ADD
    if name is not None:
        the_marker.ns = f'{name}'
        the_marker.id = id
    the_marker.lifetime = lifetime
    return the_marker    
    
def create_delete_marker(name:str, id:int, reference_frame:str):
    the_marker = create_marker(name, id, reference_frame)
    the_marker.action = Marker.DELETE
    return the_marker
    
def create_delete_all_marker(reference_frame:str):
    the_marker = Marker()
    the_marker.header.frame_id = reference_frame
    the_marker.header.stamp = rospy.Time()
    the_marker.action = Marker.DELETEALL
    return the_marker   

def create_2dregion_marker(name:str, id:int, bbox:list, offset:float, reference_frame:str, plane:str='xy', plane_thickness=0.005, rgba:list=None, lifetime=rospy.Duration(0.0)):
    the_marker = create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().CUBE
    rpy = [0, 0, 0]
    scale1 = abs(bbox[2] - bbox[0])
    scale2 = abs(bbox[3] - bbox[1])
    scale1 = 0.01 if scale1 == 0 else scale1
    scale2 = 0.01 if scale2 == 0 else scale2
    if plane == 'xy':
        xyz = [(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2, offset]
        the_marker.pose = create_pose(xyz + rpy)
        the_marker.scale = Vector3(scale1, scale2, plane_thickness)
    elif plane == 'yz':
        xyz = [offset, (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2]
        the_marker.pose = create_pose(xyz + rpy)
        the_marker.scale = Vector3(plane_thickness, scale1, scale2)
    elif plane == 'xz':
        xyz = [(bbox[0] + bbox[2]) / 2, offset, (bbox[1] + bbox[3]) / 2]
        the_marker.pose = create_pose(xyz + rpy)
        the_marker.scale = Vector3(scale1, plane_thickness, scale2)
    else:
        logger.warning(f'create_2dregion_marker: invalid plane parameter {plane}')
        return None
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker

def create_3dregion_marker(name:str, id:int, bbox:list, reference_frame:str, rgba:list=None, lifetime=rospy.Duration(0.0)):
    the_marker = create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().CUBE
    rpy = [0, 0, 0]
    xyz = [(bbox[0] + bbox[3]) / 2, (bbox[1] + bbox[4]) / 2, (bbox[2] + bbox[5]) / 2]
    the_marker.pose = create_pose(xyz + rpy)
    the_marker.scale = Vector3(bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2])
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker

def create_arrow_marker(name:str, id:int, xyzrpy:list, reference_frame:str, dimensions:list=0.5, rgba:list=None, lifetime=rospy.Duration(0.0)):
    the_marker = create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().ARROW
    the_marker.pose = create_pose(xyzrpy)
    if isinstance(dimensions, numbers.Number):
        dimensions = [dimensions, dimensions/10, dimensions/10]
    the_marker.scale = Vector3(*dimensions)
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker

def create_line_marker(name:str, id:int, xyz1:list, xyz2:list, reference_frame:str, line_width:float=0.01, rgba:list=None, lifetime=rospy.Duration(0.0)):
    the_marker = create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().LINE_STRIP
    the_marker.pose = create_pose([0, 0, 0, 0, 0, 0])
    the_marker.points[:] = [Point(*xyz1), Point(*xyz2)]
    the_marker.scale.x = line_width
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker    
    
def create_sphere_marker(name:str, id:int, xyz:list, reference_frame:str, dimensions=0.2, rgba:list=None, lifetime=rospy.Duration(0.0)):
    the_marker = create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().SPHERE 
    rpy = [0, 0, 0]
    the_marker.pose = create_pose(xyz + rpy) 
    if isinstance(dimensions, numbers.Number):
        dimensions = [dimensions, dimensions, dimensions]
    the_marker.scale = Vector3(*dimensions)
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)      
    return the_marker  

def create_text_marker(name:str, id:int, text:str, xyzrpy:list, reference_frame:str, dimensions:list=0.5, rgba:list=None, lifetime=rospy.Duration(0.0)):
    the_marker = create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().TEXT_VIEW_FACING 
    the_marker.pose = create_pose(xyzrpy)
    if isinstance(dimensions, numbers.Number):
        dimensions = [dimensions, dimensions, dimensions]
    the_marker.scale = Vector3(*dimensions)
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    the_marker.text = text
    return the_marker

class RvizVisualizer():
    def __init__(self, pub_rate=1.0):
        self.lock = threading.Lock()
        # the storage for markers
        self.marker_dict = defaultdict(lambda: None)
        self.temp_marker_list = []
        # setup marker publisher
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 100)
        # setup timer
        if pub_rate is not None and pub_rate > 0:
            self.pub_period = 1.0 / pub_rate
        else:
            self.pub_period = 1.0
        self.timer_viz = rospy.Timer(rospy.Duration(self.pub_period), self._cb_timer_viz)

    def add_persistent_marker(self, marker:Marker):
        with self.lock:
            self.marker_dict[marker.ns, marker.id] = marker
            return marker
            
    def pub_temporary_marker(self, marker:Marker):
        with self.lock:
            self.temp_marker_list.append(marker)      
            return marker     
        
    def delete_all_persistent_markers(self, frame):
        with self.lock:
            self.marker_dict.clear()
            self.temp_marker_list.append(create_delete_all_marker(frame))
            
    def delete_persistent_markers(self, name, id, frame):
        with self.lock:
            if (name, id) in self.marker_dict:
                del self.marker_dict[name, id]
                self.temp_marker_list.append(create_delete_marker(name, id, frame))    

    def _cb_timer_viz(self, event):
        with self.lock:     
            for key in self.marker_dict.keys():
                marker = self.marker_dict.get(key)
                self.marker_pub.publish(marker)
            for marker in self.temp_marker_list:
                self.marker_pub.publish(marker) 
            self.temp_marker_list.clear()                   
            
if __name__ == '__main__':
    rospy.loginfo(f'running test_rv_node')
    rospy.init_node('test_rv_node', anonymous=False)    
    rv = RvizVisualizer()
    text_marker = rv.add_persistent_marker(create_text_marker('text', 1, 'Hello', [0, 0, 0, 0.2, 0, 0], 'world', 0.3))
    rv.add_persistent_marker(create_line_marker('line', 1, [1, 0, 0], [0, 0, 1], 'world', 0.01, rgba=[0.0, 1.0, 1.0, 1.0]))
    rv.add_persistent_marker(create_sphere_marker('sphere', 1, [1, 1, 1], 'world', 0.05, rgba=[0.5, 1.0, 1.0, 1.0]))    
    rv.pub_temporary_marker(create_arrow_marker('arrow', 1, [1.0, 0.0, 0.0, 1.0, 0.0, 0.0], 'world', lifetime=rospy.Duration(3.0)))
    rospy.sleep(rospy.Duration(2.0))
    # rv.delete_all_persistent_markers('world')
    for i in range(100):
        pose = text_marker.pose
        pose.position.x += random.uniform(-0.5, 0.5)
        rospy.sleep(rospy.Duration(0.2))
    rv.delete_persistent_markers('text', 1, 'world')
    rospy.spin()