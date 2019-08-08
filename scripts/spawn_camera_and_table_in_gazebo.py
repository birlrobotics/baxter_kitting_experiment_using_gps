#!/usr/bin/env python

from baxter_kitting_experiment_using_gps import CONSTANT
import os
import subprocess
import rospy
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from tf.transformations import euler_matrix, euler_from_matrix
import numpy as np
import math

delete_model_service = None

def setup_experiment_table():
    model_name = 'experiment_table'
    delete_model_service.call(DeleteModelRequest(model_name=model_name))

    table_urdf_xacro = os.path.join(CONSTANT.gazebo_model_dir, 'table.urdf.xacro')
    table_urdf = subprocess.check_output("rosrun xacro xacro %s"%table_urdf_xacro, shell=True)
    rospy.set_param("table_urdf", table_urdf)
    subprocess.call("rosrun gazebo_ros spawn_model -urdf -param table_urdf -model %s -reference_frame baxter::base -x 0 -y 0 -z -0.7"%(model_name,), shell=True)

def setup_camera():
    model_name = 'experiment_camera'

    camera_sdf = os.path.join(CONSTANT.gazebo_model_dir, 'camera.sdf')
    delete_model_service.call(DeleteModelRequest(model_name=model_name))

    x, y, z = [0.3, 0, 0.2]
    roll, pitch, yaw = [0, 0.65, 0]

    subprocess.call("rosrun gazebo_ros spawn_model -sdf -file %s -model %s -reference_frame baxter::base -x %s -y %s -z %s -R %s -P %s -Y %s"%(camera_sdf, model_name, x, y, z, roll, pitch, yaw,), shell=True)


    rviz_frame_correction_rpy = [-math.pi/2, 0, -math.pi/2]

    tf_roll, tf_pitch, tf_yaw = euler_from_matrix(np.dot(euler_matrix(roll, pitch, yaw), euler_matrix(*rviz_frame_correction_rpy)))

    subprocess.Popen('rosrun tf static_transform_publisher %s %s %s %s %s %s /base /baxter_kitting_expertiment_camera_frame 1000'%(x, y+0.06, z, tf_yaw, tf_pitch, tf_roll,), shell=True)

def setup_object():
    model_name = 'experiment_box'
    delete_model_service.call(DeleteModelRequest(model_name=model_name))

    box_urdf_xacro = os.path.join(CONSTANT.gazebo_model_dir, 'box.urdf.xacro')
    box_urdf = subprocess.check_output("rosrun xacro xacro %s"%box_urdf_xacro, shell=True)
    rospy.set_param("box_urdf", box_urdf)
    subprocess.call("rosrun gazebo_ros spawn_model -urdf -param box_urdf -model %s -reference_frame baxter::base -x 0.7 -y 0 -z 1"%(model_name,), shell=True)

if __name__ == '__main__':
    rospy.init_node('spawn_camera_and_table_in_gazebo_node')
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    setup_experiment_table()
    setup_camera()
    setup_object()

    rospy.spin()
