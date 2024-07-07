# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import math, os
import rospy, tf, rospkg
import tf.transformations 
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

class PackageFile():
    """ The class provides tools for dealing with the uri for ros resource retriever
    """
    
    def resolve_to_file_uri(uri_or_path:str) -> str:
        """ Converts a uri or a file path into a uri starting with file:// 

        :param uri_or_path: a file, package or http uri or a full or package relative file path
        :type uri_or_path: str
        :return: the file prototcol uri of type str
        :raise: Exception if the parameter is of an invalid format
        """
        if uri_or_path is None:
            raise Exception(f'Parameter (uri_or_path) is None')
        try:
            resolved_path = PackageFile._parse(uri_or_path)
            return 'file://' + resolved_path
        except:
            raise
        
    def resolve_to_file_or_http_uri(uri_or_path:str) -> str:
        """ Converts a uri or a file path into a uri starting with file:// or http://

        :param uri_or_path: a file, package or http uri or a full or package relative file path
        :type uri_or_path: str
        :return: the file prototcol uri
        :rtype: str
        :raise: Exception if the parameter is of an invalid format
        """
        if uri_or_path.startswith(('http://', 'https://')):
            return uri_or_path
        return PackageFile.resolve_to_file_uri(uri_or_path)

    def resolve_to_path(uri_or_path:str) -> str:
        """Converts a uri or a file path into a full local file path
        
        :param uri_or_path: a file, package or http uri or a full or package relative file path
        :type uri_or_path: str
        :return: the absolute file path
        :rtype: str
        :raise: Exception if the parameter is of an invalid format
        """
        if uri_or_path is None:
            raise Exception(f'Parameter (uri_or_path) is None')
        try:
            resolved_path = PackageFile._parse(uri_or_path)
            return resolved_path
        except:
            raise
    
    # -----------------------------------
    # internal function: searching if the relative path leads to an existing file
    def _search_rospkgs_for_file(path):
        ros_package = rospkg.RosPack()
        package_path_list = ros_package.get_ros_paths()      
        for package_path in package_path_list:
            candidate_path = os.path.join(package_path, path)
            if os.path.isfile(candidate_path):
                return candidate_path
        return None
    
    # internal function: parse the input uri or file path
    def _parse(uri_or_path):
        if uri_or_path.startswith('file://localhost'):
            path = uri_or_path[16:]  
        elif uri_or_path.startswith('file://'):
            path = uri_or_path[7:]
        elif uri_or_path.startswith('package://'):
            path = uri_or_path[10:]
        elif uri_or_path.startswith(('http://', 'https://')):
            raise Exception(f'Unsupported protocol: http or https')
        elif uri_or_path.find('://') > 0:
            raise Exception(f'Unsupported protocol in uri: {uri_or_path}')
        else:
            path = uri_or_path
        if len(path) == 0:
            raise Exception(f'Malformed uri or path: empty path')
        if path[0] != '/': # path relative to package
            path = PackageFile._search_rospkgs_for_file(path)
            if path is None:
                raise Exception('No file is found by prepending ros package directories to the partial path')
        return path

if __name__ == '__main__':
    rospy.init_node('test_node')
    # test conversion functions
    tests = ['/home/qcr/arm_commander_ws/src/task_trees/demos/rviz_display/utah_teapot.stl',
            'task_trees/demos/rviz_display/utah_teapot.stl',
            'file:///home/qcr/arm_commander_ws/src/task_trees/demos/rviz_display/utah_teapot.stl',
            'file://localhost/home/qcr/arm_commander_ws/src/task_trees/demos/rviz_display/utah_teapot.stl',
            'package://task_trees/demos/rviz_display/utah_teapot.stl',
            ]
    for test in tests:
        print(f'resolve_to_fileuri: {test}\n -> {PackageFile.resolve_to_file_uri(test)}')
        print(f'resolve_to_path: {test}\n -> {PackageFile.resolve_to_path(test)}')              
