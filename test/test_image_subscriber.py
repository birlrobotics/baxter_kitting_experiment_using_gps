import rospy
import pdb
from sensor_msgs.msg import CompressedImage
from PIL import Image
import io
import numpy as np
from matplotlib import pyplot as plt



if __name__ == '__main__':
    rospy.init_node('test_node')

    while not rospy.is_shutdown():
        msg = rospy.wait_for_message('/gps_rgb_camera/image_raw/compressed', CompressedImage)
        image = Image.open(io.BytesIO(msg.data))
        image.show()
        print 'hit enter to get next frame'
        raw_input()
