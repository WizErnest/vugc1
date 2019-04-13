#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from time import sleep
from vugc1_perception.srv import Detection
from vugc1_perception.msg import *

visualization_markers_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
available = True


def delete_markers():
    markerArray = MarkerArray()
    delete_marker = Marker()
    delete_marker.action = delete_marker.DELETEALL
    markerArray.markers.append(delete_marker)
    visualization_markers_publisher.publish(markerArray)


def get_bounding_boxes(rgb):
    rospy.wait_for_service('vugc1_perception_detection')
    try:
        detection = rospy.ServiceProxy('vugc1_perception_detection', Detection)
        # xmins, xmaxs, ymins, ymaxs = detection(rgb)
        # return xmins, xmaxs, ymins, ymaxs
        result = detection(rgb)
        print('received: {}'.format(result))
        return result
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

    print('calling service...!!!')
    result = get_bounding_boxes(rgb)
    # xmins, xmaxs, ymins, ymaxs = get_bounding_boxes(rgb)
    # print('xmins = {}'.format(xmins))
    # print('ymins = {}'.format(ymins))
    # print('xmaxs = {}'.format(xmaxs))
    # print('ymaxs = {}'.format(ymaxs))

    # delete all boxes first TODO: track with uids and update
    delete_markers()

    # show the detected pedestrians
    markerArray = MarkerArray()
    for i in range(5): # for (x, y, id) in computed:
        pedestrian_marker = get_pedestrian_marker(i, i, i)
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
