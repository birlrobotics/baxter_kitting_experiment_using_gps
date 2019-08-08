#!/usr/bin/env python

from baxter_kitting_experiment_using_gps import CONSTANT
import os
import subprocess
import rospy
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest

def setup_experiment_table():
    model_name = 'experiment_table'
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    delete_model_service.call(DeleteModelRequest(model_name=model_name))

    table_urdf_xacro = os.path.join(CONSTANT.gazebo_model_dir, 'table.urdf.xacro')
    table_urdf = subprocess.check_output("rosrun xacro xacro %s"%table_urdf_xacro, shell=True)
    rospy.set_param("table_urdf", table_urdf)
    subprocess.call("rosrun gazebo_ros spawn_model -urdf -param table_urdf -model %s -reference_frame baxter::base -x 0 -y 0 -z -0.7"%(model_name,), shell=True)

def setup_camera():
    model_name = 'experiment_camera'
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    camera_sdf = os.path.join(CONSTANT.gazebo_model_dir, 'camera.sdf')
    delete_model_service.call(DeleteModelRequest(model_name=model_name))
    subprocess.call("rosrun gazebo_ros spawn_model -sdf -file %s -model %s -reference_frame baxter::base -x 0.2 -y 0 -z 0.2 -P 0.65"%(camera_sdf, model_name,), shell=True)

if __name__ == '__main__':
    rospy.init_node('spawn_camera_and_table_in_gazebo_node')
    setup_experiment_table()
    setup_camera()



