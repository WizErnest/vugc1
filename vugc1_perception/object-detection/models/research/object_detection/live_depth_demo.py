#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import cv2 as cv

from distutils.version import StrictVersion
from collections import defaultdict
from io import StringIO

# import matplotlib; matplotlib.use('Agg')  # pylint: disable=multiple-statements

#from matplotlib import pyplot as plt
from PIL import Image

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
from object_detection.utils import ops as utils_ops

#if StrictVersion(tf.__version__) < StrictVersion('1.12.0'):
#  raise ImportError('Please upgrade your TensorFlow installation to v1.12.*.')

from utils import label_map_util

from utils import visualization_utils as vis_util


# # Model preparation

# ## Variables
#
# Any model exported using the `export_inference_graph.py` tool can be loaded here simply by changing `PATH_TO_FROZEN_GRAPH` to point to a new .pb file.
#
# By default we use an "SSD with Mobilenet" model here. See the [detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) for a list of other models that can be run out-of-the-box with varying speeds and accuracies.

# What model to download.
MODEL_NAME = 'faster_rcnn_resnet101_kitti_2018_01_28' # 'ssd_mobilenet_v1_coco_2017_11_17'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('data', 'kitti_label_map.pbtxt')




# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

#infile = sys.argv[1] if len(sys.argv) >= 2 else '03-01-19-pedestrian.mp4'
#print('Reading from', infile)

#video = cv.VideoCapture(infile)
#success, image = video.read()

#inputs = []
#count = 0
#while success:
#    inputs.append(image)
#    success, image = video.read()

#inputs = np.array(inputs)
#print('Video image array shape: ', inputs.shape)

detection_graph = tf.Graph()
with detection_graph.as_default():
  config = tf.ConfigProto()
  config.gpu_options.allow_growth = True
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
	serialized_graph = fid.read()
	od_graph_def.ParseFromString(serialized_graph)
	tf.import_graph_def(od_graph_def, name='')
  sess = tf.Session(config=config)
  ops = tf.get_default_graph().get_operations()
  all_tensor_names = {output.name for op in ops for output in op.outputs}
  tensor_dict = {}
  for key in ['num_detections', 'detection_boxes', 'detection_scores',
		'detection_classes', 'detection_masks']:
	tensor_name = key + ':0'
	if tensor_name in all_tensor_names:
	  tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)


def detect_objects(image_np, sess, detection_graph):
	if 'detection_masks' in tensor_dict:
		# The following processing is only for single image
		detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
		detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
		# Reframe is required to translate mask from box coordinates to
		# image coordinates and fit the image size.
		real_num_detection = tf.cast(
		    tensor_dict['num_detections'][0], tf.int32)
		detection_boxes = tf.slice(detection_boxes, [0, 0], [
		                           real_num_detection, -1])
		detection_masks = tf.slice(detection_masks, [0, 0, 0], [
		                           real_num_detection, -1, -1])
		detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(detection_masks, detection_boxes, image.shape[0], image.shape[1])
		detection_masks_reframed = tf.cast(tf.greater(detection_masks_reframed, 0.5), tf.uint8)
		# Follow the convention by adding back the batch dimension
		tensor_dict['detection_masks'] = tf.expand_dims(detection_masks_reframed, 0)
	image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
	#output_dict = run_inference_for_single_image(image_np, detection_graph)

	output_dict = sess.run(tensor_dict, feed_dict={image_tensor: np.expand_dims(image_np, 0)})

	# all outputs are float32 numpy arrays, so convert types as appropriate
	output_dict['num_detections'] = int(output_dict['num_detections'][0])
	output_dict['detection_classes'] = output_dict[
	'detection_classes'][0].astype(np.uint8)
	output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
	output_dict['detection_scores'] = output_dict['detection_scores'][0]
	if 'detection_masks' in output_dict:
	    output_dict['detection_masks'] = output_dict['detection_masks'][0]
    
	ret_image = image_np.copy()
	vis_util.visualize_boxes_and_labels_on_image_array(ret_image, output_dict['detection_boxes'], output_dict['detection_classes'], output_dict['detection_scores'], category_index, instance_masks = output_dict.get('detection_masks'), use_normalized_coordinates = True, line_thickness = 8)

	return ret_image, output_dict['detection_boxes']

from cv_bridge import CvBridge, CvBridgeError
#from keras.models import load_model
from sensor_msgs.msg import Image
from vugc1_control.msg import drive_param
from std_msgs.msg import String
import argparse
import cv2 as cv
import numpy as np
import rospy
import time
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
control_drive_parameters = rospy.Publisher('asdf', String, queue_size=10)

# left camera FHD parameters
cx = 1068.34
cy = 513.374
fx = 1396.72
fy = 1396.72
k1 = -0.0975497
k2 = 3.49484e-06

def offhook():
    pass


def callback(message):
    print("[#callback]: received message", message.header.stamp)
    try:
		image = bridge.imgmsg_to_cv2(message)[:,:,0:3]
		start = time.time()
		with detection_graph.as_default():
			print('intermediate', time.time() - start)
			image, boxes = detect_objects(image, sess, detection_graph)
		print('time', time.time() - start)
		cv.imshow('image', image)
		cv.waitKey(1)
		#ymin, xmin, ymax, xmax = box
		print('boxes', boxes)
		
        #cv2.imwrite("demo.png", image)
		print("image of shape", image.shape, "processed")
		control_drive_parameters.publish(String("foo"))

    except CvBridgeError as e:
        print(e)

def main():
    try:
		# ## Load a (frozen) Tensorflow model into memory.


		rospy.on_shutdown(offhook)
		rospy.init_node('vugc1_control_behavioral_cloning', anonymous=True)
		rospy.Subscriber("zed/rgb/image_rect_color", Image, callback, queue_size =1)
		rospy.spin()
    finally:
	    sess.close()

if __name__=='__main__':
    main()

#outfile = sys.argv[2] if len(sys.argv)==3 else 'out.avi'
#width = 1920
#height = 1080
#print('Writing video to', outfile)
#out = cv.VideoWriter('out.avi', cv.VideoWriter_fourcc('M','J','P','G'), 30, (width,height))

#for i in res:
#    out.write(i)
#out.release()

