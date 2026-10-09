# Author: ML lab continuum-actuation team 
# University of California, San Diego

import numpy as np 
import struct 

# import different ROS message types 
import sensor_msgs.msg 
import geometry_msgs.msg
import std_msgs.msg

import PyKDL

# custom function imports 
from actuation_control import PyKDL_helper  

def convert_np_array_to_JointPosition(ral,list,joint_names):
    # TODO: naming is misleading -- msg.position has to be a list! not numpy array! 
    js_msg = sensor_msgs.msg.JointState()
    js_msg_header = std_msgs.msg.Header()
    js_msg_header.stamp = ral.now().to_msg()
    js_msg_header.frame_id = "base_link" # TODO: does this make sense? 
    js_msg.header = js_msg_header
    js_msg.name = joint_names

    js_msg.position = list
    
    return js_msg

def convert_np_array_to_JointState(ral,pos_list, effort_list, joint_names):
    # TODO: naming is misleading -- msg.position has to be a list! not numpy array! 
    js_msg = sensor_msgs.msg.JointState()
    js_msg_header = std_msgs.msg.Header()
    js_msg_header.stamp = ral.now().to_msg()
    js_msg_header.frame_id = "base_link" # TODO: does this make sense? 
    js_msg.header = js_msg_header
    js_msg.name = joint_names

    js_msg.position = pos_list
    js_msg.effort = effort_list

    return js_msg

def convert_np_pos_to_PointStamped(ral,pos):

    # convert to PoseStamped msg 
    ps_msg = geometry_msgs.msg.PointStamped()
    ps_msg.header.stamp = ral.now().to_msg()
    ps_msg.header.frame_id = "world"
    ps_msg.point.x = pos[0]
    ps_msg.point.y = pos[1]
    ps_msg.point.z = pos[2]

    return ps_msg 

def convert_float_to_PointStamped(ral,num):

    ps_msg = geometry_msgs.msg.PointStamped()
    ps_msg.header.stamp = ral.now().to_msg()
    ps_msg.header.frame_id = "world"
    ps_msg.point.x = num
    ps_msg.point.y = 0.0
    ps_msg.point.z = 0.0

    return ps_msg 


def convert_np_tf_to_PoseStamped(ral,T,frame_id = "world"):

    # convert numpt matrix to PyKDL frame 
    frame = PyKDL_helper.convert_np_tf_to_PyKDL_frame(T)

    # convert to PoseStamped msg 
    ps_msg = geometry_msgs.msg.PoseStamped()
    ps_msg.header.stamp = ral.now().to_msg()
    ps_msg.header.frame_id = frame_id
    ps_msg.pose.position.x = frame.p[0]
    ps_msg.pose.position.y = frame.p[1]
    ps_msg.pose.position.z = frame.p[2]
    ps_msg.pose.orientation.x, ps_msg.pose.orientation.y,\
        ps_msg.pose.orientation.z, ps_msg.pose.orientation.w = frame.M.GetQuaternion()

    return ps_msg 



def convert_PoseStamped_to_PyKDL(PoseStampedMsg):

    return PyKDL.Frame(PyKDL.Rotation.Quaternion(PoseStampedMsg.pose.orientation.x,
                                                PoseStampedMsg.pose.orientation.y,
                                                PoseStampedMsg.pose.orientation.z,
                                                PoseStampedMsg.pose.orientation.w),
                        PyKDL.Vector(PoseStampedMsg.pose.position.x,
                                    PoseStampedMsg.pose.position.y,
                                    PoseStampedMsg.pose.position.z))


def convert_np_tf_to_TransformStamped(ral,T,frame_name):
    # convert numpt matrix to PyKDL frame 
    frame = PyKDL_helper.convert_np_tf_to_PyKDL_frame(T)
    tf_msg = geometry_msgs.msg.TransformStamped()
    tf_msg.header.stamp = ral.now().to_msg()
    tf_msg.header.frame_id = "world"
    tf_msg.child_frame_id = frame_name
    tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, \
        tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = frame.M.GetQuaternion()
    tf_msg.transform.translation.x = frame.p[0]
    tf_msg.transform.translation.y = frame.p[1]
    tf_msg.transform.translation.z = frame.p[2]

    return tf_msg


def convert_np_tf_to_PoseArray(ral, T_stack):
    """
    Convert a stack of 4x4 numpy transforms (4,4,N)
    into a geometry_msgs/PoseArray message.

    Parameters
    ----------
    ral : rclpy node wrapper (provides ral.now())
    T_stack : np.ndarray of shape (4,4,N)

    Returns
    -------
    PoseArray
    """

    pose_array_msg = geometry_msgs.msg.PoseArray()

    # Header
    pose_array_msg.header.stamp = ral.now().to_msg()
    pose_array_msg.header.frame_id = "world"

    poses = []

    N = T_stack.shape[2]

    for i in range(N):

        # Convert numpy -> PyKDL frame
        frame = PyKDL_helper.convert_np_tf_to_PyKDL_frame(T_stack[:, :, i])

        pose = geometry_msgs.msg.Pose()

        # Orientation (quaternion)
        qx, qy, qz, qw = frame.M.GetQuaternion()
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # Position
        pose.position.x = frame.p[0]
        pose.position.y = frame.p[1]
        pose.position.z = frame.p[2]

        poses.append(pose)

    pose_array_msg.poses = poses

    return pose_array_msg

def convert_np_array_to_PointCloud2(ral,np_matrix): # TODO: reorganize -- should be in a convertMessage.py helper script 
    '''
    assumes array is a mx3 numpy array, where m is the number of coordinates defining the point cloud 
    '''
    pc2_msg = sensor_msgs.msg.PointCloud2()
    pc2_msg.header.stamp = ral.now().to_msg()
    pc2_msg.header.frame_id = 'world'

    fields = [
    sensor_msgs.msg.PointField(name='x', offset=0,  datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
    sensor_msgs.msg.PointField(name='y', offset=4,  datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
    sensor_msgs.msg.PointField(name='z', offset=8,  datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
    ]

    data = []
    for pt in np_matrix:
        data.append(struct.pack('fff', *pt))
    data = b''.join(data)

    pc2_msg.height = 1
    pc2_msg.width = np_matrix.shape[0]
    pc2_msg.fields = fields
    pc2_msg.is_bigendian = False
    pc2_msg.point_step = 12  # 3 * 4 bytes
    pc2_msg.row_step = pc2_msg.point_step * np_matrix.shape[0]
    pc2_msg.is_dense = True
    pc2_msg.data = data

    return pc2_msg 

