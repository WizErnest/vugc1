#!/usr/bin/env python

import rospy
from vugc1_control.msg import steering_values
from vugc1_control.msg import drive_param
from std_msgs.msg import Bool
from numpy import interp


control_torque_parameters = rospy.Publisher('vugc1_control_torque_parameters', steering_values, queue_size=10)
control_emergency_stop = rospy.Publisher('vugc1_control_emergency_stop', Bool, queue_size=10)

diff_low = -0.4
diff_high = 0.4
diff_mid = 0.5 * (diff_high - diff_low)
voltage_center = 2.5
str_low = -100
str_high = 100
trq_low = (4095 / 5.0) * 2.25
trq_high = (4095 / 5.0) * 2.75


def angle_2_volt(angle):
    diff = interp(angle, [str_low, str_high], [diff_low, diff_high])
    diff_half = 0.5 * diff
    vt_1 = 2.5 + diff_half
    vt_2 = 2.5 - diff_half
    return vt_1, vt_2

def callback(data):
    angle = data.angle

    print('[vugc1_control_torque_talker#callback] angle={}'.format(angle))
    if angle < -100 or angle > 100:
      print('[vugc1_control_torque_talker#callback] invalid parameters')
      return

    volts = angle_2_volt(angle)
    print('volts={}'.format(volts))
    tv_1, tv_2 = volts

    trq1 = interp(tv_1, [voltage_center + diff_low, voltage_center + diff_high], [trq_low, trq_high])
    trq2 = interp(tv_2, [voltage_center + diff_low, voltage_center + diff_high], [trq_low, trq_high])

    parameters = steering_values()
    parameters.trq_1 = trq1
    parameters.trq_2 = trq2

    print('parameters={}'.format(parameters))
    control_torque_parameters.publish(parameters)


def main():
    rospy.init_node('vugc1_control_torque_talker', anonymous=True)
    control_emergency_stop.publish(False)
    rospy.Subscriber('vugc1_control_drive_parameters', drive_param, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc1_control_torque_talker] initialized')
    main()
