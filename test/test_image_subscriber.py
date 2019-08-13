import rospy
import pdb
from sensor_msgs.msg import CompressedImage
from PIL import Image
import io
import numpy as np

def cb(msg):
    image = Image.open(io.BytesIO(msg.data))
    image_mat = np.array(image)
    pdb.set_trace()

if __name__ == '__main__':
    rospy.init_node('test_node')

    rospy.Subscriber('/baxter_kitting_expertiment_camera/image_raw/compressed', CompressedImage, cb)

    rospy.spin()
