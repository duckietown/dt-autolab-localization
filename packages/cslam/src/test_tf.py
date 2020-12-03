#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from dt_communication_utils import DTCommunicationGroup
from autolab_msgs.msg import \
    AutolabTransform

rospy.init_node('asd')
br = tf2_ros.TransformBroadcaster()

group = DTCommunicationGroup("/autolab/tf", AutolabTransform)


def cb(msg, _):
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = msg.target.name
    t.transform = msg.transform

    br.sendTransform(t)


print('Listening...')
group.Subscriber(cb)

rospy.on_shutdown(group.shutdown)

rospy.spin()

