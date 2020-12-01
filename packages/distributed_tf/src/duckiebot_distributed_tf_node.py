#!/usr/bin/env python3

import re
import rospy
import tf2_ros
import threading

from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup
from autolab_msgs.msg import \
    AutolabTransform, \
    AutolabReferenceFrame


class DistributedTFNode(DTROS):

    FETCH_TF_STATIC_EVERY_SECS = 10
    PUBLISH_TF_STATIC_EVERY_SECS = 10

    def __init__(self):
        super(DistributedTFNode, self).__init__(
            node_name='distributed_tf_node',
            node_type=NodeType.SWARM
        )
        # get static parameter - `~veh`
        try:
            self.robot_hostname = rospy.get_param('~veh')
        except KeyError:
            self.logerr('The parameter ~veh was not set, the node will abort.')
            exit(1)
        # get static parameter - `~map`
        try:
            self.map_name = _sanitize_hostname(rospy.get_param('~map'))
        except KeyError:
            self.logerr('The parameter ~map was not set, the node will abort.')
            exit(2)
        # get static parameter - `~tag_id`
        try:
            self.tag_id = rospy.get_param('~tag_id')
        except KeyError:
            self.logerr('The parameter ~tag_id was not set, the node will abort.')
            exit(3)
        # create communication group
        self._group = DTCommunicationGroup("/autolab/tf", AutolabTransform)
        # create static tfs holder and access semaphore
        self._static_tfs = {
            (f'{self.robot_hostname}/footprint', f'tag/{self.tag_id}'): None
        }
        self._static_tfs_sem = threading.Semaphore(1)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # create publishers
        self._tf_pub = self._group.Publisher()
        self._tf_static_timer = rospy.Timer(
            rospy.Duration(self.FETCH_TF_STATIC_EVERY_SECS),
            self._fetch_tfs
        )
        self._pub_timer = rospy.Timer(
            rospy.Duration(self.PUBLISH_TF_STATIC_EVERY_SECS),
            self._publish_tfs
        )
        # create publishers
        self._tf_pub = self._group.Publisher()

    def on_shutdown(self):
        if hasattr(self, '_group') and self._group is not None:
            self._group.shutdown()

    def _fetch_tfs(self, *_):
        origin = f'tag/{self.tag_id}'
        target = f'{self.robot_hostname}/footprint'
        # try to fetch the TF
        try:
            transform = self._tf_listener.lookupTransform(origin, target)
            tf = AutolabTransform(
                origin=AutolabReferenceFrame(
                    time=transform.header.stamp,
                    type=AutolabReferenceFrame.ENTITY_TYPE_DUCKIEBOT_TAG,
                    name=origin,
                    robot=self.robot_hostname
                ),
                target=AutolabReferenceFrame(
                    time=transform.header.stamp,
                    type=AutolabReferenceFrame.ENTITY_TYPE_DUCKIEBOT_FOOTPRINT,
                    name=target,
                    robot=self.robot_hostname
                ),
                is_fixed=False,
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
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    def _publish_tfs(self, *_):
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


def _sanitize_hostname(s: str):
    return re.sub("[^0-9a-zA-Z]+", "", s)


if __name__ == '__main__':
    node = DistributedTFNode()
    # spin forever
    rospy.spin()
