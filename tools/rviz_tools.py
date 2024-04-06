# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

# References for implementation of create_pointcloud_from_image: 
# https://raw.githubusercontent.com/DavidB-CMU/rviz_tools_py/master/src/rviz_tools_py/rviz_tools.py
# https://github.com/eric-wieser/ros_numpy/tree/master/src/ros_numpy
# https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb


__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import yaml, os, time, numbers, threading, random, struct
from enum import Enum
from collections import defaultdict
import cv2
import numpy as np
import rospy, tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3, Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker
from tools.pose_tools import list_to_pose
from tools.logging_tools import logger

class RGBAColors(int, Enum):
    """ Define common use colours for visualization

    """
    RED = 0, (1.0, 0.0, 0.0, 0.5)
    BLUE = 1, (0.0, 0.0, 1.0, 0.5)
    GREEN = 2, (0.0, 1.0, 0.0, 0.5)
    def __new__(cls, value, rgba='...'):
        obj = int.__new__(cls, value)
        obj._value_ = value
        obj.rgba = rgba
        return obj
    
def _create_marker(name:str, id:int, reference_frame:str=None, lifetime=rospy.Duration(0.0)) -> Marker:
    """ Create a Marker object
    :meta private:
    :param name: the name space of the marker
    :param id: the id of the marker
    :param reference_frame: the reference frame, defaults to None
    :param lifetime: the duration that the marker is displayed, defaults to rospy.Duration(0.0)
    :return: Marker
    """
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
    """ Returns a Marker object specified to delete a marker

    :param name: the name space of the marker
    :param id: the id of the marker
    :param reference_frame: the reference frame, defaults to None
    :return: the Marker object for deleting a marker
    """
    the_marker = _create_marker(name, id, reference_frame)
    the_marker.action = Marker.DELETE
    return the_marker
    
def create_delete_all_marker(reference_frame:str):
    """ Returns a Marker object specified to delete all markers

    :param reference_frame: the reference frame, defaults to None
    :return: the Marker object for deleting all markers
    """
    the_marker = Marker()
    the_marker.header.frame_id = reference_frame
    the_marker.header.stamp = rospy.Time()
    the_marker.action = Marker.DELETEALL
    return the_marker   

def create_2dregion_marker(name:str, id:int, bbox:list, offset:float, reference_frame:str, plane:str='xy', plane_thickness=0.005, rgba:list=None, lifetime=rospy.Duration(0.0)) -> Marker:
    """ Creates a marker for displaying a 2D region as a plane

    :param name: the name space of the marker
    :param id: the id of the marker
    :param bbox: a bounding box as a list [min_x, min_y, max_x, max_y]
    :param offset: the z value where the plane is display
    :param reference_frame: the reference frame, defaults to None
    :param plane: a string representing the plane where the bounding box lies, defaults to 'xy'
    :param plane_thickness: the thickness of the plane to be displayed, defaults to 0.005
    :param rgba: the colour and alpha value, defaults to None
    :param lifetime: the duration that the marker is displayed, defaults to rospy.Duration(0.0)
    :return: the Marker object
    """
    the_marker = _create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().CUBE
    rpy = [0, 0, 0]
    scale1 = abs(bbox[2] - bbox[0])
    scale2 = abs(bbox[3] - bbox[1])
    scale1 = 0.01 if scale1 == 0 else scale1
    scale2 = 0.01 if scale2 == 0 else scale2
    if plane == 'xy':
        xyz = [(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2, offset]
        the_marker.pose = list_to_pose(xyz + rpy)
        the_marker.scale = Vector3(scale1, scale2, plane_thickness)
    elif plane == 'yz':
        xyz = [offset, (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2]
        the_marker.pose = list_to_pose(xyz + rpy)
        the_marker.scale = Vector3(plane_thickness, scale1, scale2)
    elif plane == 'xz':
        xyz = [(bbox[0] + bbox[2]) / 2, offset, (bbox[1] + bbox[3]) / 2]
        the_marker.pose = list_to_pose(xyz + rpy)
        the_marker.scale = Vector3(scale1, plane_thickness, scale2)
    else:
        logger.warning(f'create_2dregion_marker: invalid plane parameter {plane}')
        return None
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker

def create_3dregion_marker(name:str, id:int, bbox:list, reference_frame:str, rgba:list=None, lifetime=rospy.Duration(0.0)) -> Marker:
    """ Creates a marker for displaying a 3D region as a box

    :param name: the name space of the marker
    :param id: the id of the marker
    :param bbox: a bounding box as a list [min_x, min_y, min_z, max_x, max_y, max_z]
    :param reference_frame: the reference frame, defaults to None
    :param rgba: the colour and alpha value, defaults to None
    :param lifetime: the duration that the marker is displayed, defaults to rospy.Duration(0.0)
    :return: the Marker object
    """
    the_marker = _create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().CUBE
    rpy = [0, 0, 0]
    xyz = [(bbox[0] + bbox[3]) / 2, (bbox[1] + bbox[4]) / 2, (bbox[2] + bbox[5]) / 2]
    the_marker.pose = list_to_pose(xyz + rpy)
    the_marker.scale = Vector3(bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2])
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker

def create_arrow_marker(name:str, id:int, xyzrpy:list, reference_frame:str, dimensions:list=0.5, rgba:list=None, lifetime=rospy.Duration(0.0)) -> Marker:
    """ Creates a marker for displaying an arrow

    :param name: the name space of the marker
    :param id: the id of the marker
    :param xyzrpy: the pose of the arrow as a list of 6
    :param reference_frame: the reference frame, defaults to None
    :param dimensions: the thickness of the arrow, defaults to 0.5
    :param rgba: the colour and alpha value, defaults to None
    :param lifetime: the duration that the marker is displayed, defaults to rospy.Duration(0.0)
    :return: the Marker object
    """
    the_marker = _create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().ARROW
    the_marker.pose = list_to_pose(xyzrpy)
    if isinstance(dimensions, numbers.Number):
        dimensions = [dimensions, dimensions/10, dimensions/10]
    the_marker.scale = Vector3(*dimensions)
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker

def create_line_marker(name:str, id:int, xyz1:list, xyz2:list, reference_frame:str, line_width:float=0.01, rgba:list=None, lifetime=rospy.Duration(0.0)) -> Marker:
    """ Creates a marker for displaying a line

    :param name: the name space of the marker
    :param id: the id of the marker
    :param xyz1: the first point of the line
    :param xyz2: the second point of the line
    :param reference_frame: the reference frame, defaults to None
    :param line_width: the width of the line, defaults to 0.01
    :param rgba: the colour and alpha value, defaults to None
    :param lifetime: the duration that the marker is displayed, defaults to rospy.Duration(0.0)
    :return: the Marker object
    """
    the_marker = _create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().LINE_STRIP
    the_marker.pose = list_to_pose([0, 0, 0, 0, 0, 0])
    the_marker.points[:] = [Point(*xyz1), Point(*xyz2)]
    the_marker.scale.x = line_width
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    return the_marker    
    
def create_sphere_marker(name:str, id:int, xyz:list, reference_frame:str, dimensions=0.2, rgba:list=None, lifetime=rospy.Duration(0.0)) -> Marker:
    """ Creates a marker for displaying a sphere

    :param name: the name space of the marker
    :param id: the id of the marker
    :param xyz: the position of the sphere
    :param reference_frame: the reference frame, defaults to None
    :param dimensions: the dimensions of the sphere as a list of 3 dimensions or a number, defaults to 0.2
    :param rgba: the colour and alpha value, defaults to None
    :param lifetime: the duration that the marker is displayed, defaults to rospy.Duration(0.0)
    :return: the Marker object
    """
    the_marker = _create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().SPHERE 
    rpy = [0, 0, 0]
    the_marker.pose = list_to_pose(xyz + rpy) 
    if isinstance(dimensions, numbers.Number):
        dimensions = [dimensions, dimensions, dimensions]
    the_marker.scale = Vector3(*dimensions)
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)      
    return the_marker  

def create_text_marker(name:str, id:int, text:str, xyzrpy:list, reference_frame:str, dimensions:list=0.5, rgba:list=None, lifetime=rospy.Duration(0.0)) -> Marker:
    """ Creates a marker for displaying a text

    :param name: the name space of the marker
    :param id: the id of the marker
    :param text: a string to be displayed
    :param xyzrpy: the pose of the text as a list of 6
    :param reference_frame: the reference frame, defaults to None
    :param dimensions: the size of the text, defaults to 0.5
    :param rgba: the colour and alpha value, defaults to None
    :param lifetime: the duration that the marker is displayed, defaults to rospy.Duration(0.0)
    :return: the Marker object
    """
    the_marker = _create_marker(name, id, reference_frame, lifetime)
    the_marker.type = Marker().TEXT_VIEW_FACING 
    the_marker.pose = list_to_pose(xyzrpy)
    if isinstance(dimensions, numbers.Number):
        dimensions = [dimensions, dimensions, dimensions]
    the_marker.scale = Vector3(*dimensions)
    rgba = RGBAColors.RED.rgba if rgba is None else rgba
    the_marker.color = ColorRGBA(*rgba)
    the_marker.text = text
    return the_marker

def create_pointcloud_from_image(image_bgr, xyz:list=(0, 0, 0), pixel_physical_size:float=0.005, reference_frame=None, opacity=255, depth_array=None) -> PointCloud2:
    """ Create a PointCloud2 for displaying a OpenCV image (color or greyscale) 

    :param image_bgr: the image to be displayed, type numpy array
    :param xyz: the position of the bottom left hand corner of the image, defaults to (0, 0, 0)
    :param pixel_physical_size: the length of each pixel in x, y, defaults to [0.005, 0.005], and optionally the third value in the list for z scaling factor
    :param reference_frame: the reference frame, defaults to None
    :param opacity: the opacity of the displayed image, defaults to 255
    :param depth_array: optionally a numpy array of exact the same shape as the image indicating the depth, defaults to None
    :return: the PointCloud2 object
    """

    if depth_array is not None:
        if image_bgr.shape[0] != depth_array.shape[0] or image_bgr.shape[1] != depth_array.shape[1]:
            logger.error(f'{__name__} (image_to_pointcloud): the shape of the parameter depth_array {depth_array.shape} is different from the image_bgr') 
            raise AssertionError('Parameters have different dimensions')
    # fill xyz with default values if it is not a list of 3 numbers
    if xyz is None or type(xyz) not in (list, tuple):
        xyz = [0, 0, 0]
    elif type(xyz) is tuple:
        xyz = list(xyz)
    for _ in range(len(xyz), 3):
        xyz.append(0)
    # fill pixel_physical_size with default values
    default_pixel_physical_size = [0.005, 0.005, 1]
    if pixel_physical_size is None:
        pixel_physical_size = default_pixel_physical_size
    elif isinstance(pixel_physical_size, numbers.Number):
        pixel_physical_size = [pixel_physical_size, pixel_physical_size, 1]
    elif type(pixel_physical_size) in (list, tuple):
        pixel_physical_size = list(pixel_physical_size)
        for i in range(len(pixel_physical_size), 3):
            pixel_physical_size.append(default_pixel_physical_size[i])
    # prepare data structures
    is_grey = len(image_bgr.shape) == 2
    image_height, image_width = image_bgr.shape[0], image_bgr.shape[1]
    num_pixels = image_height * image_width
    if is_grey:
        cloud_data = np.zeros(num_pixels, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('value', np.uint8)])
    else:
        cloud_data = np.zeros(num_pixels, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
    # compute the point location for every pixel, for x and y, they are computed from the pixel position scaled by the pixel physical size 
    cloud_data['x'] = np.tile(np.linspace(0, image_width, image_width) * pixel_physical_size[0] + xyz[0], image_height)
    cloud_data['y'] = ((np.repeat(np.linspace(0, image_height, image_height) * pixel_physical_size[1], image_width) - image_height * pixel_physical_size[1]) * -1 + xyz[1])
    if depth_array is None:
        cloud_data['z'] = np.full(num_pixels, xyz[2])
    else:
        cloud_data['z'] = np.reshape(depth_array * pixel_physical_size[2], num_pixels)
    # combine the pixel values into a numpy array of shape (num_pixels, 4) for both greyscale and rgb images
    if is_grey:
        fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.UINT8, 1),]
        cloud_data['value'] = np.reshape(image_bgr, num_pixels)
    else:
        fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1),]
        r = np.asarray(np.reshape(image_bgr[:, :, 2], num_pixels), dtype=np.uint32)
        g = np.asarray(np.reshape(image_bgr[:, :, 1], num_pixels), dtype=np.uint32)
        b = np.asarray(np.reshape(image_bgr[:, :, 0], num_pixels), dtype=np.uint32)   
        cloud_data['rgb'] = np.array((opacity << 24) | (r << 16) | (g << 8) | (b << 0), dtype=np.uint32) 
    # create a PointCloud2 message using the data
    cloud_point_list = cloud_data.tolist()
    return point_cloud2.create_cloud(Header(frame_id = reference_frame), fields, cloud_point_list)


# --------------------------------------------
# Models a publisher of markers
class RvizVisualizer():
    """ A publisher of markers, which handles persistent markers, which is published repeatedly and temporary markers,
        which are published once.
    """
    def __init__(self, **kwargs):
        """ The constructur

        :param pub_rate: the rate of publishing, defaults to 1.0
        """
        self.lock = threading.Lock()
        # set default values for keyword argument
        pub_rate_marker = kwargs.get('pub_rate_marker', 1.0)
        topic_marker = kwargs.get('topic_marker', '/visualization_marker')
        pub_rate_cloud = kwargs.get('pub_rate_cloud', 1.0)
        topic_cloud = kwargs.get('topic_cloud', '/visualization_cloud')
        # the storage for markers
        self.marker_dict = defaultdict(lambda: None)
        self.temp_marker_list = []
        self.pointcloud_dict = defaultdict(lambda: None)
        # setup marker publisher
        self.marker_pub = rospy.Publisher(topic_marker, Marker, queue_size = 100)
        self.cloud_pub = rospy.Publisher(topic_cloud, PointCloud2, queue_size = 2)
        # setup timer
        pub_period_marker = 1.0 / pub_rate_marker if pub_rate_marker > 0 else 1.0
        self.timer_marker_viz = rospy.Timer(rospy.Duration(pub_period_marker), self._cb_timer_marker_viz)
        pub_period_cloud = 1.0 / pub_rate_cloud if pub_rate_cloud > 0 else 1.0
        self.timer_cloud_viz = rospy.Timer(rospy.Duration(pub_period_cloud), self._cb_timer_cloud_viz)

    def _cb_timer_marker_viz(self, event):
        """ internal callback function 
        :meta private:
        """
        with self.lock:     
            for key in self.marker_dict.keys():
                marker = self.marker_dict.get(key)
                self.marker_pub.publish(marker)
            for marker in self.temp_marker_list:
                self.marker_pub.publish(marker) 
            self.temp_marker_list.clear() 

    def _cb_timer_cloud_viz(self, event):
        """ internal callback function 
        :meta private:
        """
        with self.lock:   
            for pointcloud in self.pointcloud_dict.values():
                pointcloud.header.stamp = rospy.Time.now()
                self.cloud_pub.publish(pointcloud)

    def add_persistent_marker(self, marker:Marker) -> Marker:
        """ Add a persistent marker

        :param marker: A marker to be persistently published
        :return: The mrker
        """
        with self.lock:
            self.marker_dict[marker.ns, marker.id] = marker
            return marker
            
    def pub_temporary_marker(self, marker:Marker) -> Marker:
        """ Add a temporary marker

        :param marker: A marker to be published only once
        :return: The mrker
        """
        with self.lock:
            self.temp_marker_list.append(marker)      
            return marker     
        
    def delete_all_persistent_markers(self, frame) -> None:
        """ Remove all persistent markers from RViz and this object

        :param frame: The reference frame
        """
        with self.lock:
            self.marker_dict.clear()
            self.temp_marker_list.append(create_delete_all_marker(frame))
            
    def delete_persistent_markers(self, name, id, frame) -> None:
        """ Remove a persistent marker from RViz and this object

        :param name: the name space of the marker
        :param id: the id of the marker
        :param frame: the reference frame
        """
        with self.lock:
            if (name, id) in self.marker_dict:
                del self.marker_dict[name, id]
                self.temp_marker_list.append(create_delete_marker(name, id, frame))    

    def add_pointcloud(self, name, pointcloud:PointCloud2) -> PointCloud2:
        """ Add a PointCloud2 object for regular publishing

        :param name: the name of the pointcloud of type str
        :param pointcloud: an object to be published
        :return: the point cloud input parameter 
        """
        with self.lock:
            self.pointcloud_dict[name] = pointcloud
            return pointcloud

    def delete_pointcloud(self, name) -> PointCloud2:
        """ Remove a pointcloud from regular publishing

        :param name: the name of the pointcloud of type str
        :return: the PointCloud2 object removed
        """
        with self.lock:
            if name in self.pointcloud_dict:
                pointcloud = self.pointcloud_dict[name]  
                del self.pointcloud_dict[name]        
                return pointcloud
            return None

# -- the test program
if __name__ == '__main__':
    rospy.loginfo(f'running test_rv_node')
    rospy.init_node('test_rv_node', anonymous=False)    
    # test the mark visualization
    rv = RvizVisualizer()
    text_marker = rv.add_persistent_marker(create_text_marker('text', 1, 'Hello', [0, 0, 0, 0.2, 0, 0], 'world', 0.3))
    rv.add_persistent_marker(create_line_marker('line', 1, [1, 0, 0], [0, 0, 1], 'world', 0.01, rgba=[0.0, 1.0, 1.0, 1.0]))
    rv.add_persistent_marker(create_sphere_marker('sphere', 1, [1, 1, 1], 'world', 0.05, rgba=[0.5, 1.0, 1.0, 1.0]))    
    rv.pub_temporary_marker(create_arrow_marker('arrow', 1, [1.0, 0.0, 0.0, 1.0, 0.0, 0.0], 'world', lifetime=rospy.Duration(3.0)))
    rospy.sleep(rospy.Duration(2.0))
    # rv.delete_all_persistent_markers('world')
    # for i in range(100):
    #     pose = text_marker.pose
    #     pose.position.x += random.uniform(-0.5, 0.5)
    #     rospy.sleep(rospy.Duration(0.2))
    # rv.delete_persistent_markers('text', 1, 'world')
    # test the image visualization
    image_bgr = cv2.imread(os.path.join(os.path.dirname(__file__), '../sample.jpg'))
    pc2_message = create_pointcloud_from_image(image_bgr, (1, 0.5, 1), pixel_physical_size=[0.001, 0.002, -1], reference_frame='world')
    rv.add_pointcloud('the_image', pc2_message)
    rospy.spin()