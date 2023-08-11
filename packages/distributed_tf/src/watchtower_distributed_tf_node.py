#!/usr/bin/env python3

import os
import re

import yaml
import rospy
import rospkg

from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup
from duckietown_msgs.msg import \
    AprilTagDetectionArray
from autolab_msgs.msg import \
    AutolabTransform, \
    AutolabReferenceFrame


class DistributedTFNode(DTROS):

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
        # load atags DB
        # TODO: this is temporary, we should actually subscribe to the atags_with_info topic
        #       we can't do that now because the atag-post-processor node is a mess
        self._tags = self._load_atag_db()
        self._tag_type_to_rf_type = {
            'Localization': AutolabReferenceFrame.TYPE_GROUND_TAG,
            'Vehicle': AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG
        }
        self._tag_type_to_is_static = {
            'Localization': True,
            'Vehicle': False
        }
        # create communication group
        self._group = DTCommunicationGroup("/autolab/tf", AutolabTransform)
        # create publishers
        self._tf_pub = self._group.Publisher()
        # create local subscribers
        self._atag_sub = rospy.Subscriber(
            '~detections_in',
            AprilTagDetectionArray,
            self._cb_atag,
            queue_size=20
        )

    def on_shutdown(self):
        if hasattr(self, '_group') and self._group is not None:
            self._group.shutdown()

    def _cb_atag(self, msg):
    
    # compute variance using msg.center[0] (x value in img frame) and msg.center[1] (y value in img frame)
    # height = 972 width = 1296     (center "0,0": 648, 486)
    # msg.pose_error could also be used
    
        for detection in msg.detections:
            if detection.tag_id not in self._tags:
                continue
            tag_type = self._tags[detection.tag_id]["tag_type"]
            if tag_type not in self._tag_type_to_rf_type:
                continue
            rf_type = self._tag_type_to_rf_type[tag_type]
            is_static = self._tag_type_to_is_static[tag_type]
            # ---
            tf = AutolabTransform(
                origin=AutolabReferenceFrame(
                    time=msg.header.stamp,
                    type=AutolabReferenceFrame.TYPE_WATCHTOWER_CAMERA,
                    name=msg.header.frame_id,
                    robot=self.robot_hostname
                ),
                target=AutolabReferenceFrame(
                    time=msg.header.stamp,
                    type=rf_type,
                    name=f"tag/{detection.tag_id}",
                    robot=self.map_name
                ),
                is_fixed=False,
                is_static=is_static,
                transform=detection.transform,
                variance=(1/810) * np.sqrt((detection.center[0] - 648)**2 + (detection.center[1] - 486)**2)
                #added variance to be a number from 0-1 based on the location of the april tag in the image
                #further away from the center of the image = higher variance
            )
            # publish
            self._tf_pub.publish(tf, destination=self.map_name)

    @staticmethod
    def _load_atag_db():
        # TODO: this is temporary, we should actually subscribe to the atags_with_info topic
        #       we can't do that now because the atag-post-processor node is a mess
        rospack = rospkg.RosPack()
        distributed_tf_pkg_path = rospack.get_path('distributed_tf')
        apriltags_db_filepath = os.path.join(distributed_tf_pkg_path, '..', '..', 'assets', 'apriltagsDB.yaml')
        tags = {}
        with open(apriltags_db_filepath, 'r') as fin:
            db = yaml.safe_load(fin)
            for tag in db:
                tags[tag["tag_id"]] = tag
        return tags


def _sanitize_hostname(s: str):
    return re.sub("[^0-9a-zA-Z]+", "", s)


if __name__ == '__main__':
    node = DistributedTFNode()
    # spin forever
    rospy.spin()
