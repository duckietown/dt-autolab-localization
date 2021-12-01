#!/usr/bin/env python3

import curses

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from dt_communication_utils import DTCommunicationGroup
from autolab_msgs.msg import \
    AutolabTransform

# rospy.init_node('autolab_tf_listener')
# br = tf2_ros.TransformBroadcaster()

group = DTCommunicationGroup("/autolab/tf", AutolabTransform)
shelf = set()

# terminal repainting
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

def cb(msg, _):
    shelf.add((msg.origin.name, msg.target.name))

    # t = TransformStamped()
    #
    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = msg.origin.name
    # t.child_frame_id = msg.target.name
    # t.transform = msg.transform
    #
    # br.sendTransform(t)

    # print sorted edges
    for i, e in enumerate(sorted(shelf)):
        # pad some space to clear potentially longer previous outputs
        stdscr.addstr(i, 0, str(e) + ' ' * 40)  
    stdscr.refresh()


try:
    group.Subscriber(cb)
finally:
    curses.echo()
    curses.nocbreak()
    curses.endwin()


# rospy.on_shutdown(group.shutdown)
#
# rospy.spin()

