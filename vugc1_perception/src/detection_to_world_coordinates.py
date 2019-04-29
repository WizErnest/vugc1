#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from time import sleep
from vugc1_perception.srv import Detection
from vugc1_perception.msg import *
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError


available = True
bridge = CvBridge()
visualization_markers_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)

cx = 692.67
cy = 345.187
fx = 698.359
fy = 698.359
k1 = -0.0975497
k2 = 3.49484e-06


def delete_markers():
    markerArray = MarkerArray()
    delete_marker = Marker()
    delete_marker.action = delete_marker.DELETEALL
    markerArray.markers.append(delete_marker)
    visualization_markers_publisher.publish(markerArray)


def get_bounding_boxes(color, depth):
    rospy.wait_for_service('vugc1_perception_detection')
    try:
        detection = rospy.ServiceProxy('vugc1_perception_detection', Detection)
        result = detection(color, depth)
        # print('received: {}'.format(result))
        return result.boxes
    except rospy.ServiceException, e:
        print('service call failed: {}'.format(e))
        return None, None, None, None


def get_pedestrian_marker(id, x, y):
    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.ns = 'vugc1_perception'
    marker.id = id
    marker.type = marker.CYLINDER
    marker.action = marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 1.5

    marker.color.g = 1.0
    marker.color.a = 1.0

    return marker


def callback(rgb, depth):
    global available
    if not available:
        pass
    available = False

    print('[#callback]: rgb={}'.format(rgb.header.stamp))
    print('[#callback]: depth={}'.format(depth.header.stamp))
    print('[#callback]: calling service...')
    boxes = get_bounding_boxes(rgb, depth)

    # delete all boxes first TODO: track with uids and update
    delete_markers()

    markerArray = MarkerArray()
    # show the detected pedestrians
    for i, box in enumerate(list(boxes.boxes)):
        print('x=({}, {}) y=({}, {}) depth={}'.format(box.xmin, box.xmax, box.ymin, box.ymax, box.depth))
        # centers of bounding box in pixel coordinates
        x_center = 720 * ((box.xmin + box.xmax) / 2.0)
        y_center = 1280 * ((box.ymin + box.ymax) / 2.0)
        depth = box.depth

        X = (1 / fx) * (x_center - cx)
        Y = (1 / fy) * (y_center - cy)
        true_y = depth ** 2 - X ** 2

        pedestrian_marker = get_pedestrian_marker(i, true_y, -X)
        markerArray.markers.append(pedestrian_marker)

    visualization_markers_publisher.publish(markerArray)
    available = True


def main():
    rospy.init_node('vugc1_perception_detection_to_world_coordinates', anonymous=True)

    rgb = message_filters.Subscriber('zed/rgb/image_rect_color', Image, queue_size=1)
    depth = message_filters.Subscriber('zed/depth/depth_registered', Image, queue_size=1)

    ts = message_filters.TimeSynchronizer([rgb, depth], 1)
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    main()
