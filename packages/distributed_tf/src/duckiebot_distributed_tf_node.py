#!/usr/bin/env python3

import re
import rospy
import tf2_ros
import threading
import numpy as np

from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup
from autolab_msgs.msg import \
    AutolabTransform, \
    AutolabReferenceFrame

from nav_msgs.msg import Odometry

from geometry_msgs.msg import \
    Transform, \
    Vector3, \
    Quaternion

import tf.transformations as tr


class DistributedTFNode(DTROS):

    FETCH_TF_STATIC_EVERY_SECS = 60
    PUBLISH_TF_STATIC_EVERY_SECS = 2

    def __init__(self):
        super(DistributedTFNode, self).__init__(
            node_name='distributed_tf_node',
            node_type=NodeType.SWARM
        )
        # get static parameters
        self.robot_hostname = rospy.get_param('~veh')
        self.map_name = _sanitize_hostname(rospy.get_param('~map'))
        self.tag_id = rospy.get_param('~tag_id')
        self.min_distance_odom = rospy.get_param('~min_distance_odom')
        self.max_time_between_poses = rospy.get_param('~max_time_between_poses')
        self.transforms = rospy.get_param('~transforms', [])
        # apply values to templated TFs
        for tf in self.transforms:
            tf['origin'] = tf['origin'].format(veh=self.robot_hostname, tag_id=self.tag_id)
            tf['target'] = tf['target'].format(veh=self.robot_hostname, tag_id=self.tag_id)
        # define local reference frames' names
        self._tag_frame = f'tag/{self.tag_id}'
        self._footprint_frame = f'{self.robot_hostname}/footprint'
        # create communication group
        self._group = DTCommunicationGroup("/autolab/tf", AutolabTransform)
        # create static tfs holder and access semaphore
        self._static_tfs = {
            (tf['origin'], tf['target']): AutolabTransform(
                origin=AutolabReferenceFrame(
                    time=rospy.Time(),
                    type=AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT,
                    name=tf['origin'],
                    robot=self.robot_hostname
                ),
                target=AutolabReferenceFrame(
                    time=rospy.Time(),
                    type=AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG,
                    name=tf['target'],
                    robot=self.robot_hostname
                ),
                is_fixed=True,
                is_static=False,
                transform=Transform(
                    translation=Vector3(*tf['translation']),
                    rotation=Quaternion(*tr.quaternion_from_euler(*tf['rotation']))
                )
            ) for tf in self.transforms
        }
        self._static_tfs_sem = threading.Semaphore(1)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # create publishers
        self._tf_pub = self._group.Publisher()
        # fetch/publish right away and then set timers
        if self.tag_id != '__NOTSET__':
            # self._fetch_static_tfs()
            self._publish_static_tfs()
            # TODO: disabled because TFs were added to the YAML config file since ROS does not like multiple URDFs combination
            # self._tf_static_timer = rospy.Timer(
            #     rospy.Duration(self.FETCH_TF_STATIC_EVERY_SECS),
            #     self._fetch_static_tfs
            # )
            self._pub_timer = rospy.Timer(
                rospy.Duration(self.PUBLISH_TF_STATIC_EVERY_SECS),
                self._publish_static_tfs
            )
        # setup subscribers for odometry
        self._odo_sub = rospy.Subscriber(
            "~odometry_in",
            Odometry,
            self._cb_odometry,
            queue_size=1
        )
        self._pose_last = None
        self._reminder = DTReminder(frequency=10)

    def on_shutdown(self):
        if hasattr(self, '_group') and self._group is not None:
            self._group.shutdown()

    def _fetch_static_tfs(self, *_):
        origin = self._tag_frame
        target = self._footprint_frame
        # try to fetch the TF
        try:
            transform = self._tf_buffer.lookup_transform(origin, target, rospy.Time())
            tf = AutolabTransform(
                origin=AutolabReferenceFrame(
                    time=transform.header.stamp,
                    type=AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG,
                    name=origin,
                    robot=self.robot_hostname
                ),
                target=AutolabReferenceFrame(
                    time=transform.header.stamp,
                    type=AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT,
                    name=target,
                    robot=self.robot_hostname
                ),
                is_fixed=True,
                is_static=False,
                transform=transform.transform
            )
            # add TF to the list of TFs to publish
            self._static_tfs_sem.acquire()
            try:
                self._static_tfs[(origin, target)] = tf
            finally:
                self._static_tfs_sem.release()
            # stop timer
            self.loginfo("All static TFs needed have been fetched. Disabling TF listener.")
            # noinspection PyBroadException
            try:
                self._tf_static_timer.shutdown()
            except BaseException:
                pass
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.logwarn(f"Could not find a static TF [{origin}] -> [{target}]")
        except tf2_ros.ConnectivityException:
            pass

    def _publish_static_tfs(self, *_):
        tfs = []
        self._static_tfs_sem.acquire()
        try:
            for transform in self._static_tfs.values():
                tfs.append(transform)
        finally:
            self._static_tfs_sem.release()
        # publish
        for tf in tfs:
            self._tf_pub.publish(tf, destination=self.map_name)

    def _cb_odometry(self, pose_now):
        if self._pose_last is None:
            self._pose_last = pose_now
            return

        if not self._reminder.is_time():
            return

        # Only add the transform if the new pose is sufficiently different
        t_now_to_world = np.array([pose_now.pose.pose.position.x,
                                   pose_now.pose.pose.position.y,
                                   pose_now.pose.pose.position.z])

        t_last_to_world = np.array([self._pose_last.pose.pose.position.x,
                                    self._pose_last.pose.pose.position.y,
                                    self._pose_last.pose.pose.position.z])

        o = pose_now.pose.pose.orientation
        q_now_to_world = np.array([o.x, o.y, o.z, o.w])

        o = self._pose_last.pose.pose.orientation
        q_last_to_world = np.array([o.x, o.y, o.z, o.w])

        q_world_to_last = q_last_to_world
        q_world_to_last[3] *= -1
        q_now_to_last = tr.quaternion_multiply(q_world_to_last, q_now_to_world)
        world_to_last = np.matrix(tr.quaternion_matrix(q_world_to_last))

        #now_to_last = np.matrix(tr.quaternion_matrix(q_now_to_last))
        # print(now_to_last)
        #now_to_last = now_to_last[0:3][:, 0:3]
        R_world_to_last = world_to_last[0:3][:, 0:3]
        # print(now_to_last)
        #t_now_to_last = np.array(np.dot(now_to_last, t_now_to_world - t_last_to_world))
        t_now_to_last = np.array(np.dot(R_world_to_last, t_now_to_world - t_last_to_world))

        t_now_to_last = t_now_to_last.flatten()

        # compute TF between `pose_now` and `pose_last`
        transform = Transform(
            translation=Vector3(x=t_now_to_last[0],
                                y=t_now_to_last[1],
                                z=t_now_to_last[2]),
            rotation=Quaternion(x=q_now_to_last[0],
                                y=q_now_to_last[1],
                                z=q_now_to_last[2],
                                w=q_now_to_last[3])
        )

        # pack TF into an AutolabTransform message
        tf = AutolabTransform(
            origin=AutolabReferenceFrame(
                time=self._pose_last.header.stamp,
                type=AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT,
                name=self._footprint_frame,
                robot=self.robot_hostname
            ),
            target=AutolabReferenceFrame(
                time=pose_now.header.stamp,
                type=AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT,
                name=self._footprint_frame,
                robot=self.robot_hostname
            ),
            is_fixed=False,
            is_static=False,
            transform=transform
        )
        # publish
        self._tf_pub.publish(tf, destination=self.map_name)
        # update last pose
        self._pose_last = pose_now


def _sanitize_hostname(s: str):
    return re.sub("[^0-9a-zA-Z]+", "", s)


if __name__ == '__main__':
    node = DistributedTFNode()
    # spin forever
    rospy.spin()
