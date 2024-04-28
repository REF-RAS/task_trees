# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import math
import rospy, tf
import tf.transformations 
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

def list_to_pose(pose) -> Pose:
    """ Convert a pose in xyzrpy or xyzq format to Pose

    :param pose: s pose in a list format of xyzrpy or xyzq
    :return: a Pose object with the same pose
    """
    xyzq = list_to_xyzq(pose)
    position = Point(*xyzq[:3])    
    orientation = Quaternion(*xyzq[3:])        
    return Pose(position, orientation)

def list_to_pose_stamped(pose, frame) -> PoseStamped:
    """ Convert a pose in xyzrpy or xyzq format in a reference frame to PoseStamped

    :param pose: a pose in a list format of xyzrpy or xyzq
    :param frame: the reference frame of type str
    :return: a Pose object with the same pose
    """
    if frame is None:
        raise AssertionError(f'{__name__} (list_to_pose_stamped): parameter frame is None')
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.pose = list_to_pose(pose)
    return pose_stamped

def pose_to_xyzq(pose) -> list:
    """ Convert a pose in Pose or PoseStamped format to xyzq format

    :param pose: the pose of type Pose or PoseStamped
    :return: a list of the format xyzqqqq with the same pose    
    """
    if pose is None:
        raise AssertionError(f'{__name__} (pose_to_xyzq): parameter pose is None')
    if type(pose) == PoseStamped:
        pose = pose.pose
    if type(pose) != Pose:
        raise AssertionError(f'{__name__} (pose_to_xyzq): parameter pose type should be Pose or PoseStamped')
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def pose_to_xyzrpy(pose) -> list:
    """ Convert a pose in Pose or PoseStamped format into xyzrpy format

    :param pose: the pose of type Pose or PoseStamped
    :return: a list of the format xyzrpy with the same pose 
    """
    xyzq = pose_to_xyzq(pose)
    rpy = list(tf.transformations.euler_from_quaternion(xyzq[3:]))
    return xyzq[:3] + rpy

def list_to_xyzq(pose) -> list:
    """ Convert a pose in xyzrpy or xyzq format to xyzq format

    :param pose: a pose in a list format of xyzrpy or xyzq
    :return: a list of the format xyzqqqq with the same pose     
    """
    if pose is None or type(pose) not in (list, tuple) or len(pose) not in (6, 7):
        raise AssertionError(f'{__name__} (list_to_pose): parameter pose is not a list of 6 or 7 numbers')
    if len(pose) == 6: 
        return pose[:3]  + list(tf.transformations.quaternion_from_euler(*pose[3:]))
    else:
        return pose

def list_to_xyzrpy(pose) -> list:
    """ Convert a pose in xyzrpy or xyzq format to xyzrpy format

    :param pose: a pose in a list format of xyzrpy or xyzq
    :return: a list of the format xyzrpy with the same pose 
    """
    if pose is None or type(pose) not in (list, tuple) or len(pose) not in (6, 7):
        raise AssertionError(f'{__name__} (list_to_pose): parameter pose is not a list of 6 or 7 numbers')  
    if len(pose) == 6:
        return pose
    else:
        return pose[:3] + list(tf.transformations.euler_from_quaternion(pose[3:]))

def same_rpy_with_tolerence(rpy_1:list, rpy_2:list, tolerance:float=0.01) -> bool:
    """ Test if two euler's angles are the same

    :param rpy_1: the first orientation in euler's angle
    :type rpy_1: list
    :param rpy_2: the second orientation in euler's angle
    :type rpy_2: list
    :param tolerance: the absolute tolerance for the comparison
    :type tolerance: float
    :return: True if the two orientations are deemed the same
    :rtype: bool
    """
    if len(rpy_1) != 3 or len(rpy_2) != 3:
        return False
    q1 = tf.transformations.quaternion_from_euler(*rpy_1)
    q2 = tf.transformations.quaternion_from_euler(*rpy_2)
    q1[3] = -q1[3] # inverse
    qr = tf.transformations.quaternion_multiply(q2, q1)
    rpy = tf.transformations.euler_from_quaternion(qr)
    return math.fabs(rpy[0]) + math.fabs(rpy[1]) + math.fabs(rpy[2]) < tolerance

def same_xyz_with_tolerence(xyz_1:list, xyz_2:list, tolerance:float=0.01) -> bool:
    """ Test if two positions are the same

    :param xyz_1: the first position in xyz
    :type xyz_1: list
    :param xyz_2: the second position in xyz
    :type xyz_2: list
    :param tolerance: the absolute tolerance for the comparison
    :type tolerance: float
    :return: True if the two orientations are deemed the same
    :rtype: bool
    """
    if len(xyz_1) != 3 or len(xyz_2) != 3:
        return False
    d = math.dist((xyz_1[0], xyz_1[1], xyz_1[2]), (xyz_2[0], xyz_2[1], xyz_2[2])) 
    return d < tolerance

def same_pose_with_tolerence(pose_1, pose_2, tolerance:float=0.01) -> bool:
    """ Test if two poses are the same within a tolerance of each other.
        
    :param goal: A pose
    :type goal:  A list of floats or a Pose
    :param actual: another pose
    :type actual: A list of floats or a Pose
    :param tolerance: the absolute tolerance for the comparison
    :type tolerance: float
    :return: True if the goal and actual are the same 
    :rtype: bool
    """
    if type(pose_1) is PoseStamped or type(pose_2) is PoseStamped:
        raise AssertionError(f'{__name__} (same_pose_with_tolerence): parameter pose should not be PoseStamped')
    if type(pose_1) is Pose:
        pose_1 = pose_to_xyzrpy(pose_1)
    elif type(pose_1) in (list, tuple):
        pose_1 = list_to_xyzrpy(pose_1)
    else:
        raise AssertionError(f'{__name__} (same_pose_with_tolerence): pose_1 should be Pose or list')
    if type(pose_2) is Pose:
        pose_2 = pose_to_xyzrpy(pose_2)
    elif type(pose_2) in (list, tuple):
        pose_2 = list_to_xyzrpy(pose_2)
    else:
        raise AssertionError(f'{__name__} (same_pose_with_tolerence): pose_2 should be Pose or list')
    return same_xyz_with_tolerence(pose_1[:3], pose_2[:3], tolerance) and same_rpy_with_tolerence(pose_1[3:], pose_2[3:], tolerance)

def same_joint_values_with_tolerence(joint_values_1:list, joint_values_2:list, tolerance:float=0.1) -> bool:
    """ Test if the two sets of joint values are the same

    :param joint_values_1: the first set of joint_values
    :type joint_values_1: list
    :param joint_values_2: the second set of joint values
    :type joint_values_2: list
    :param tolerance: the absolute tolerance for the comparison
    :type tolerance: float
    :return: True if the two sets of joint values are deemed the same
    :rtype: bool
    """
    if len(joint_values_1) != len(joint_values_2):
        return False
    for index in range(len(joint_values_1)):
        if abs(joint_values_1[index] - joint_values_2[index]) > tolerance:
            return False
    return True

# def in_region(x:float, y:float, bbox_as_list:list) -> bool:
#     """ Test if the point (x, y) is in the bbox
    
#     :param x: the x value of the point
#     :type x: float
#     :param y: the y value of the point
#     :type y: float
#     :param bbox_as_list: the bounding square
#     :type bbox_as_list: (x1, y1, x2, y2)
#     :return: True if the point (x, y) is within the bbox
#     :rtype: bool
#     """
#     return (bbox_as_list[0] <= x <= bbox_as_list[2]) and (bbox_as_list[1] <= y <= bbox_as_list[3])

def in_region(point:list, bbox_as_list:list) -> bool:
    """ Test if the point (x, y) or (x, y, z) is in the bbox
    
    :param point: the (x, y) or (x, y, z) value of the point
    :type point: list of 2 or 3 numbers
    :param bbox_as_list: the bounding square or bounding box
    :type bbox_as_list: (x1, y1, x2, y2) or (x1, y1, z1, x2, y2, z2)
    :return: True if the point is within the bbox
    :rtype: bool
    """
    if bbox_as_list is None or type(bbox_as_list) not in (list, tuple) or point is None or type(point) not in (list, tuple):
        return False
    if len(bbox_as_list) == 4 and len(point) >= 2:
        return (bbox_as_list[0] <= point[0] <= bbox_as_list[2]) and (bbox_as_list[1] <= point[1] <= bbox_as_list[3])
    elif len(bbox_as_list) == 6 and len(point) >= 3:
        return (bbox_as_list[0] <= point[0] <= bbox_as_list[3]) and (bbox_as_list[1] <= point[1] <= bbox_as_list[4]) and \
              (bbox_as_list[2] <= point[2] <= bbox_as_list[5])
    return False  

def overlap_regions(bbox1_as_list, bbox2_as_list):
    """ Test if two regions, 2d or 3d as bbox, are overlapped

    :param bbox1_as_list: the bounding square or bounding box
    :param bbox2_as_list: the bounding square or bounding box
    :return: True of the two regions are overlapped
    """
    if bbox1_as_list is None or type(bbox1_as_list) not in (list, tuple) or bbox2_as_list is None or type(bbox2_as_list) not in (list, tuple):
        return False
    if len(bbox1_as_list) == 4 or len(bbox2_as_list) == 4:
        if (bbox1_as_list[0] >= bbox2_as_list[2]) or (bbox1_as_list[2] <= bbox2_as_list[0]) or (bbox1_as_list[3] <= bbox2_as_list[1]) or (bbox1_as_list[1] >= bbox2_as_list[3]):
            return False
        return True
    elif len(bbox1_as_list) == 6 and len(bbox2_as_list) == 6:
        if (bbox1_as_list[0] >= bbox2_as_list[3]) or (bbox1_as_list[3] <= bbox2_as_list[0]) or (bbox1_as_list[4] <= bbox2_as_list[1]) or (bbox1_as_list[1] >= bbox2_as_list[4]) \
            or (bbox1_as_list[2] >= bbox2_as_list[5]) or (bbox1_as_list[5] <= bbox2_as_list[2]):
            return False
        return True
    return False        

if __name__ == '__main__':
    rospy.init_node('test_node')
    # test conversion functions
    xyzrpy_1 = [1.0, 2.0, 3.0, 3.14, 0.2, -3.5]
    pose = list_to_pose(xyzrpy_1)
    print(f'pose: {xyzrpy_1} {pose}')
    pose_stamped = list_to_pose_stamped(xyzrpy_1, 'world')
    print(f'poseStamped: {xyzrpy_1} {pose_stamped}')
    print(f'back to xyzrpy: {pose_to_xyzrpy(pose)}')
    print(f'back to xyzrpy: {pose_to_xyzrpy(pose_stamped)}')    
    print(f'back to xyzq: {pose_to_xyzq(pose)}')
    print(f'back to xyzq: {pose_to_xyzq(pose_stamped)}')   
    xyzq = list_to_xyzq(xyzrpy_1)
    print(f'list to xyzq: {xyzq}')       
    print(f'list to xyzrpy: {list_to_xyzrpy(xyzq)}')       
    print(f'list to xyzrpy: {list_to_xyzrpy(xyzrpy_1)}')  
    # test comparison functions (expects True)
    xyzrpy_1 = [1.0, 2.0, 3.0, 3.14, 0.2, -3.500]
    xyzrpy_2 = [1.0, 2.0, 3.0, 3.14, 0.2, 2.783]
    tolerence = 0.001
    print(f'same_rpy_with_tolerence: {same_rpy_with_tolerence(xyzrpy_1[3:], xyzrpy_2[3:], tolerence)}')
    print(f'same_xyz_with_tolerence: {same_xyz_with_tolerence(xyzrpy_1[:3], xyzrpy_2[:3], tolerence)}')
    print(f'same_pose_with_tolerence: {same_pose_with_tolerence(xyzrpy_1, xyzrpy_2, tolerence)}')
    pose_1, pose_2 = list_to_pose(xyzrpy_1), list_to_pose(xyzrpy_2)
    print(f'same_pose_with_tolerence: {same_pose_with_tolerence(pose_1, pose_2, tolerence)}')
    xyzq_1, xyzq_2 = pose_to_xyzq(pose_1), pose_to_xyzq(pose_2)
    print(f'same_pose_with_tolerence: {same_pose_with_tolerence(xyzq_1, xyzq_2, tolerence)}')    
    # test comparison functions (expects False)
    xyzrpy_1 = [1.0, 2.0, 3.0, 3.14, 0.2, -3.5]
    xyzrpy_2 = [1.0, 2.02, 3.0, 3.14, 0.22, 2.783]
    tolerence = 0.01
    print(f'same_rpy_with_tolerence: {same_rpy_with_tolerence(xyzrpy_1[3:], xyzrpy_2[3:], tolerence)}')
    print(f'same_xyz_with_tolerence: {same_xyz_with_tolerence(xyzrpy_1[:3], xyzrpy_2[:3], tolerence)}')
    print(f'same_pose_with_tolerence: {same_pose_with_tolerence(xyzrpy_1, xyzrpy_2, tolerence)}')
    pose_1, pose_2 = list_to_pose(xyzrpy_1), list_to_pose(xyzrpy_2)
    print(f'same_pose_with_tolerence: {same_pose_with_tolerence(pose_1, pose_2, tolerence)}')
    xyzq_1, xyzq_2 = pose_to_xyzq(pose_1), pose_to_xyzq(pose_2)
    print(f'same_pose_with_tolerence: {same_pose_with_tolerence(xyzq_1, xyzq_2, tolerence)}')   