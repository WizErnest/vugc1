#!/usr/bin/env python
# coding: utf-8

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import cv2 as cv
import argparse
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vugc1_perception.srv import Detection
from vugc1_perception.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import String

from distutils.version import StrictVersion
from collections import defaultdict
from io import StringIO
from PIL import Image
sys.path.append("..") # needed since the notebook is stored in the object_detection folder.
from object_detection.utils import ops as utils_ops
from utils import label_map_util
from utils import visualization_utils as vis_util


MODEL_NAME = 'faster_rcnn_resnet101_kitti_2018_01_28' # 'ssd_mobilenet_v1_coco_2017_11_17'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb' # path to frozen detection graph (actual model used for detection)
PATH_TO_LABELS = os.path.join('data', 'kitti_label_map.pbtxt') # label strings

# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
bridge = CvBridge()


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
        for key in ['num_detections', 'detection_boxes', 'detection_scores', 'detection_classes', 'detection_masks']:
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
    print('session ran')
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


def callback(message):
    print("[#callback]: received color", message.color.header.stamp)
    print("[#callback]: received depth", message.depth.header.stamp)
    try:
        color = bridge.imgmsg_to_cv2(message.color)[:,:,0:3]
        depth = bridge.imgmsg_to_cv2(message.depth)

        start = time.time()
        with detection_graph.as_default():
            print('[#callback]: starting inference', time.time() - start)
            color, boxes = detect_objects(color, sess, detection_graph)
        print('[#callback]: inference time: {}'.format(time.time() - start))

        nonzero_indices = np.where(boxes.any(axis=1))[0]

        bounding_boxes = BoundingBoxArray()
        if len(nonzero_indices)==0:
            return bounding_boxes

        boxes = boxes[nonzero_indices]
        for i in boxes:
            ymin = i[0]
            xmin = i[1]
            ymax = i[2]
            xmax = i[3]
            bounding_box = BoundingBox()
            bounding_box.xmin = xmin
            bounding_box.xmax = xmax
            bounding_box.ymin = ymin
            bounding_box.ymax = ymax

            x_center = int(1280*(xmin + xmax) / 2.0)
            y_center = int(720 * (ymin + ymax) / 2.0)
            depths = np.nan_to_num(depth[y_center - 10: y_center + 10, x_center - 10: x_center+10])

            bounding_box.depth = np.average(depths)
            bounding_boxes.boxes.append(bounding_box)

        return bounding_boxes

    except CvBridgeError as e:
        print(e)


def main():
    rospy.init_node('object_detection_server')
    service = rospy.Service('vugc1_perception_detection', Detection, callback)
    print("[live_depth_service]: ready to take image")
    rospy.spin()


if __name__=='__main__':
    main()
