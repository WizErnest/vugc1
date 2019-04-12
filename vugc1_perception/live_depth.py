from cv_bridge import CvBridge, CvBridgeError
#from keras.models import load_model
from sensor_msgs.msg import Image
from vugc1_control.msg import drive_param
import argparse
import cv2 as cv
import numpy as np
import rospy
import tensorflow as tf
#from keras.backend.tensorflow_backend import set_session
#config = tf.ConfigProto()
# config.gpu_options.per_process_gpu_memory_fraction = 0.3
# set_session(tf.Session(config=config))

# arguments
# parser = argparse.ArgumentParser(description='[behavioral cloning] choose model')
# parser.add_argument('--model', type=str, help='model')
# args = parser.parse_args()

# # model
# model = load_model(args.model)
# model._make_predict_function()

bridge = CvBridge()
control_drive_parameters = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)

# left camera FHD parameters
cx = 1068.34
cy = 513.374
fx = 1396.72
fy = 1396.72
k1 = -0.0975497
k2 = 3.49484e-06

def offhook():
    control_drive_parameters = rospy.Publisher('vugc1_control_drive_parameters', drive_param, queue_size=10)
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    control_drive_parameters.publish(message)


def callback(message):
    print("[#callback]: received message")
    try:
        image = bridge.imgmsg_to_cv2(message)
        np.save("demo.npy", image)
        print(image.shape)
        min_range = 0
        max_range = 10
        image =255 -  255/10 * image.copy()
        #cv2.imwrite("demo.png", image)
        cv.imshow('img', image)




        return
        #image = image[188:, 0:672, 0:3]
        #image = cv2.resize(image, (320, 160))

        #angle = float(model.predict(image[None, :, :, :], batch_size=1))
        #print('[#callback]: angle={}'.format(angle))

        #message = drive_param()
        #message.velocity = 16
        #message.angle = angle
        #control_drive_parameters.publish(message)
    except CvBridgeError as e:
        print(e)


def main():
    rospy.on_shutdown(offhook)
    rospy.init_node('vugc1_control_behavioral_cloning', anonymous=True)
    rospy.Subscriber("zed/depth/depth_registered", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
