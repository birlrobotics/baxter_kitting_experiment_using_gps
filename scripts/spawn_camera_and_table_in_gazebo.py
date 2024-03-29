#!/usr/bin/env python

from baxter_kitting_experiment_using_gps import CONSTANT
import os
import subprocess
import rospy
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from tf.transformations import euler_matrix, euler_from_matrix
import numpy as np
import math
import signal
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
import tf

delete_model_service = None
gazebo_links_to_pubish_to_tf = []
gazebo_links_tf_name = []

def setup_experiment_table():
    global gazebo_links_to_pubish_to_tf
    model_name = 'experiment_table'
    delete_model_service.call(DeleteModelRequest(model_name=model_name))

    table_urdf_xacro = os.path.join(CONSTANT.gazebo_model_dir, 'table.urdf.xacro')
    table_urdf = subprocess.check_output("rosrun xacro xacro %s"%table_urdf_xacro, shell=True)
    rospy.set_param("table_urdf", table_urdf)
    subprocess.call("rosrun gazebo_ros spawn_model -urdf -param table_urdf -model %s -reference_frame baxter::base -x 0 -y 0 -z -0.7"%(model_name,), shell=True)
    gazebo_links_to_pubish_to_tf.append('experiment_table::table_top_link')
    gazebo_links_tf_name.append('table_top_link')

def setup_camera():
    model_name = 'experiment_camera'

    camera_sdf = os.path.join(CONSTANT.gazebo_model_dir, 'camera.sdf')
    delete_model_service.call(DeleteModelRequest(model_name=model_name))

    x, y, z = [1.5, -0.5, 0.2]
    roll, pitch, yaw = [0, math.pi/8, math.pi*6.5/8]

    subprocess.call("rosrun gazebo_ros spawn_model -sdf -file %s -model %s -reference_frame baxter::base -x %s -y %s -z %s -R %s -P %s -Y %s"%(camera_sdf, model_name, x, y, z, roll, pitch, yaw,), shell=True)


    rviz_frame_correction_rpy = [-math.pi/2, 0, -math.pi/2]

    tf_roll, tf_pitch, tf_yaw = euler_from_matrix(np.dot(euler_matrix(roll, pitch, yaw), euler_matrix(*rviz_frame_correction_rpy)))

    tf_pub_process = subprocess.Popen('rosrun tf static_transform_publisher %s %s %s %s %s %s /base /gps_rgb_camera_frame 1000'%(x, y, z, tf_yaw, tf_pitch, tf_roll,), shell=True)

    return tf_pub_process

def setup_object():
    global gazebo_links_to_pubish_to_tf

    model_name = 'experiment_box'
    delete_model_service.call(DeleteModelRequest(model_name=model_name))

    box_urdf_xacro = os.path.join(CONSTANT.gazebo_model_dir, 'box.urdf.xacro')
    box_urdf = subprocess.check_output("rosrun xacro xacro %s"%box_urdf_xacro, shell=True)
    rospy.set_param("box_urdf", box_urdf)
    subprocess.call("rosrun gazebo_ros spawn_model -urdf -param box_urdf -model %s -reference_frame baxter::base -x 0.85 -y 0 -z 1"%(model_name,), shell=True)

    gazebo_links_to_pubish_to_tf.append('experiment_box::experiment_box_link')
    gazebo_links_tf_name.append('experiment_box_link')

if __name__ == '__main__':
    rospy.init_node('spawn_camera_and_table_in_gazebo_node')
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    setup_experiment_table()
    tf_pub_process = setup_camera()
    setup_object()

    tf_broadcaster = tf.TransformBroadcaster()
    get_gazebo_link_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        for count, gazebo_link in enumerate(gazebo_links_to_pubish_to_tf): 
            req = GetLinkStateRequest(
                link_name=gazebo_link,
                reference_frame='base'
            )
        
    
            resp = get_gazebo_link_service.call(req)

            pos = resp.link_state.pose.position
            ori = resp.link_state.pose.orientation
            tf_broadcaster.sendTransform(
                translation=(pos.x, pos.y, pos.z),
                rotation=(ori.x, ori.y, ori.z, ori.w),
                time=rospy.Time.now(),
                child=gazebo_links_tf_name[count],
                parent='base',
            )

        r.sleep()

    os.kill(tf_pub_process.pid, signal.SIGINT)
    os.kill(tf_pub_process.pid, signal.SIGTERM)
    
