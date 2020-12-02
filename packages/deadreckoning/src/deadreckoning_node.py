#!/usr/bin/env python3

import os
#from collections import defaultdict
#from threading import Semaphore
import math
import numpy as np

#import matplotlib.image as pimage
import matplotlib.pyplot as plt
import rosbag
import tf
import rospy
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion, PoseStamped
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

#from deadreckoning.utils import TransformStamped_to_TF
#from deadreckoning.utils import TF

# TODO: this is temp stuff
from deadreckoning.__temporary import _is_wheel, _is_excluded, _is_duckiebot_tag

# constants
BAG_NAME = "2020-11-20-22-52-08"

# Used for deadreckoning
WHEEL_RADIUS = 0.03275#0.0325#0.0318
WHEELBASE = 0.1
ODOM_DT_MAX = 0.05


class DeadReckoningNode:

    def __init__(self):
        # super().__init__(
        #     "cslam_node",
        #     node_type=NodeType.LOCALIZATION
        # )

        self.pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)

        # Keep a list of left and right wheel angles,
        # since they are not synchronized
        self._left_wheel_angles = []
        self._left_wheel_times = []
        self._right_wheel_angles = []
        self._right_wheel_times = []

        self._left_wheel_angle_last = None
        self._left_wheel_time_last = None
        self._right_wheel_angle_last = None
        self._right_wheel_time_last = None

        self.timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]

        # Used for debugging
        self.x_trajectory = []
        self.y_trajectory = []
        self.yaw_trajectory = []
        self.time = []

        self.total_dist = 0

    def publish_pose (self):
        pose = PoseStamped()
        pose.header.stamp = self.timestamp
        pose.header.frame_id = "TBD"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.x = self.q[0]
        pose.pose.orientation.y = self.q[1]
        pose.pose.orientation.z = self.q[2]
        pose.pose.orientation.w = self.q[3]

        #self.pub.publish(pose)


    def cb_tf(self, msg):
        if not _is_wheel(msg):
            return

        for transform in msg.transforms:
            origin = msg.transforms[0].header.frame_id
            target = msg.transforms[0].child_frame_id

            timestamp = msg.transforms[0].header.stamp.secs + msg.transforms[0].header.stamp.nsecs*1E-9

            q = msg.transforms[0].transform.rotation
            q = [q.x, q.y, q.z, q.w]
            e = tf.transformations.euler_from_quaternion(q)

            timestamp_avg = None
            tv = None
            rv = None


            if target.endswith('left_wheel'):

                left_wheel_q = q

                if len(node._right_wheel_times):
                    idx = min(range(len(node._right_wheel_times)), key=lambda i: abs(node._right_wheel_times[i]-timestamp))

                    dt = abs(node._right_wheel_times[idx]-timestamp)
                    if dt < ODOM_DT_MAX:

                        right_wheel_q = node._right_wheel_angles[idx]
                        left_wheel_timestamp = timestamp
                        right_wheel_timestamp = node._right_wheel_times[idx]


                        if node._left_wheel_angle_last and node._right_wheel_angle_last:

                            node._left_wheel_angle_last[3] *= -1
                            qrel_left = tf.transformations.quaternion_multiply(node._left_wheel_angle_last, left_wheel_q)
                            e_left =  tf.transformations.euler_from_quaternion(qrel_left)

                            distance_left = e_left[1] * WHEEL_RADIUS


                            node._right_wheel_angle_last[3] *= -1
                            qrel_right = tf.transformations.quaternion_multiply(node._right_wheel_angle_last, right_wheel_q)
                            e_right = tf.transformations.euler_from_quaternion(qrel_right)

                            distance_right = e_right[1] * WHEEL_RADIUS

                            distance = (distance_left + distance_right)/2
                            yaw_angle = (distance_right - distance_left)/WHEELBASE

                            print('distance left: ' + f'{distance_left}; distance_right: ' + f'{distance_right}')

                            timestamp_avg = (left_wheel_timestamp + right_wheel_timestamp)/2

                            timestamp_avg_last = (node._left_wheel_time_last + node._right_wheel_time_last)/2

                            tv = distance/(timestamp_avg - timestamp_avg_last)
                            rv = yaw_angle/(timestamp_avg - timestamp_avg_last)

                            print('TV: ' + f'{tv} RV: ' + f'{rv};  DT: ' + f'{timestamp_avg - timestamp_avg_last}')

                        node._left_wheel_time_last = left_wheel_timestamp
                        node._left_wheel_angle_last = left_wheel_q

                        node._right_wheel_time_last = right_wheel_timestamp
                        node._right_wheel_angle_last = right_wheel_q

                        node._right_wheel_angles.pop(idx)
                        node._right_wheel_times.pop(idx)
                    else:
                        #print('Left: DT = ' + f'{dt}')
                        node._left_wheel_times.append(timestamp)
                        node._left_wheel_angles.append(left_wheel_q)
                else:
                    node._left_wheel_times.append(timestamp)
                    node._left_wheel_angles.append(left_wheel_q)


            elif target.endswith('right_wheel'):

                right_wheel_q = q

                if len(node._left_wheel_times):
                    idx = min(range(len(node._left_wheel_times)), key=lambda i: abs(node._left_wheel_times[i]-timestamp))


                    dt = abs(node._left_wheel_times[idx]-timestamp)
                    if dt < ODOM_DT_MAX:

                        left_wheel_q = node._left_wheel_angles[idx]
                        right_wheel_timestamp = timestamp
                        left_wheel_timestamp = node._left_wheel_times[idx]

                        if node._right_wheel_angle_last and node._left_wheel_angle_last:

                            node._right_wheel_angle_last[3] *= -1
                            qrel_right = tf.transformations.quaternion_multiply(node._right_wheel_angle_last, right_wheel_q)
                            e_right = tf.transformations.euler_from_quaternion(qrel_right)

                            distance_right = e_right[1] * WHEEL_RADIUS


                            node._left_wheel_angle_last[3] *= -1
                            qrel_left = tf.transformations.quaternion_multiply(node._left_wheel_angle_last, left_wheel_q)
                            e_left = tf.transformations.euler_from_quaternion(qrel_left)

                            distance_left = e_left[1] * WHEEL_RADIUS

                            distance = (distance_right + distance_left)/2
                            yaw_angle = (distance_left - distance_right)/WHEELBASE

                            print('distance right: ' + f'{distance_right}; distance_left: ' + f'{distance_left}')

                            timestamp_avg = (right_wheel_timestamp + left_wheel_timestamp)/2

                            timestamp_avg_last = (node._right_wheel_time_last + node._left_wheel_time_last)/2

                            tv = distance/(timestamp_avg - timestamp_avg_last)
                            rv = yaw_angle/(timestamp_avg - timestamp_avg_last)

                            print('TV: ' + f'{tv} RV: ' + f'{rv};  DT: ' + f'{timestamp_avg - timestamp_avg_last}')

                        node._right_wheel_time_last = right_wheel_timestamp
                        node._right_wheel_angle_last = right_wheel_q

                        node._left_wheel_time_last = left_wheel_timestamp
                        node._left_wheel_angle_last = left_wheel_q

                        node._left_wheel_angles.pop(idx)
                        node._left_wheel_times.pop(idx)
                    else:
                        node._right_wheel_times.append(timestamp)
                        node._right_wheel_angles.append(right_wheel_q)
                else:
                    node._right_wheel_times.append(timestamp)
                    node._right_wheel_angles.append(right_wheel_q)

            if node.timestamp and timestamp_avg:
                timestamp_last = node.timestamp

                dist = tv*(timestamp_avg - timestamp_last)
                angl = rv*(timestamp_avg - timestamp_last)

                # Assume zero roll and pitch
                qrel = tf.transformations.quaternion_from_euler (0.0, 0.0, angl)
                node.q = tf.transformations.quaternion_multiply(node.q, qrel)
                rpy = tf.transformations.euler_from_quaternion(node.q)
                yaw = rpy[2]

                node.x = node.x + dist*math.cos(yaw)
                node.y = node.y + dist*math.sin(yaw)
                node.timestamp = timestamp_avg

                # For debugging
                node.yaw_trajectory.append(yaw)
                node.x_trajectory.append(node.x)
                node.y_trajectory.append(node.y)
                node.time.append(timestamp_avg)

                node.total_dist = node.total_dist + dist

                print('Total dist: ' + f'{node.total_dist}')

            elif timestamp_avg:
                node.timestamp = timestamp_avg


            node.publish_pose()

            #print('List lengths: Left: ' + f'{len(node._left_wheel_times)}; Right: ' + f'{len(node._right_wheel_times)}')


def angle_clamp (theta):

    if (theta > 2*math.pi):
        return theta - 2*math.pi
    elif (theta < -2*math.pi):
        return theta + 2*math.pi;
    else:
        return theta


if __name__ == '__main__':

    # Initialize the node with rospy
    #rospy.init_node('DeadReckoningNode', anonymous=False)

    bag_filename = f"{BAG_NAME}.bag"
    bag_filepath = os.path.join(os.environ.get("DT_REPO_PATH"), "assets", "logs", bag_filename)
    # create node
    node = DeadReckoningNode()
    topic_to_cb = {
        "/tf": node.cb_tf,
    }

    qright_last = None
    with rosbag.Bag(bag_filepath) as bag:
        for topic, msg, msg_time in bag.read_messages(topics=list(topic_to_cb.keys())):
            if not _is_wheel(msg):
                continue

            # TODO: this is temporary, should go when the devices are time-synced
            #for i in range(len(msg.transforms)):
            #    msg.transforms[i].header.stamp = msg_time
            # fake-publish message
            topic_to_cb[topic](msg)


    plt.figure(1)
    plt.plot(node.x_trajectory, node.y_trajectory, 'bo')
    plt.axes().set_aspect('equal')
    plt.show()
    # ---
    # rospy.signal_shutdown("done")
