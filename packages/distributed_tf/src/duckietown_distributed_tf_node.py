#!/usr/bin/env python3

import rospy
import geometry
import numpy as np

import duckietown_world as dw

from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup
from autolab_msgs.msg import \
    AutolabTransform, \
    AutolabReferenceFrame
from geometry_msgs.msg import \
    Transform, \
    Vector3, \
    Quaternion


class DistributedTFNode(DTROS):

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
        # make sure the map exists
        maps = dw.list_maps()
        if self.robot_hostname not in maps:
            self.logerr(f"Map `{self.robot_hostname}` not found in "
                        f"duckietown-world=={dw.__version__}. "
                        f"The node will abort.")
            exit(2)
        self._map = dw.load_map(self.robot_hostname)
        # create communication group
        self._group = DTCommunicationGroup("/autolab/tf", AutolabTransform)
        # create TF between the /world frame and the origin of this map
        self._world_to_map_origin_tf = AutolabTransform(
            origin=AutolabReferenceFrame(
                time=rospy.Time.now(),
                type=AutolabReferenceFrame.ENTITY_TYPE_WORLD,
                name="world",
                robot="*"
            ),
            target=AutolabReferenceFrame(
                time=rospy.Time.now(),
                type=AutolabReferenceFrame.ENTITY_TYPE_MAP_ORIGIN,
                name="map",
                robot=self.robot_hostname
            ),
            is_fixed=True,
            is_static=False,
            transform=Transform(
                translation=Vector3(x=0, y=0, z=0),
                rotation=Quaternion(x=0, y=0, z=0, w=1)
            )
        )
        # create static tfs holder and access semaphore
        self._static_tfs = [
            self._world_to_map_origin_tf
        ]
        # add TFs from ground tags
        self._static_tfs.extend(self._get_tags_tfs())
        # create publisher
        self._tf_pub = self._group.Publisher()
        self._pub_timer = rospy.Timer(
            rospy.Duration(self.PUBLISH_TF_STATIC_EVERY_SECS),
            self._publish_tfs
        )

    def on_shutdown(self):
        if hasattr(self, '_group') and self._group is not None:
            self._group.shutdown()

    def _publish_tfs(self, *_):
        # publish
        for tf in self._static_tfs:
            self._tf_pub.publish(tf, destination=self.robot_hostname)

    def _get_tags_tfs(self):
        tfs = []
        # iterate over the floor tags
        for srel in self._map.spatial_relations.values():
            try:
                origin, (target,) = srel.a, srel.b
                if origin != ():
                    continue
                if not target.startswith('tag'):
                    continue
                position = srel.transform.p
                theta = srel.transform.theta
                # get 3D rotation
                Rz = geometry.rotation_from_axis_angle(np.array([0, 0, 1]), np.deg2rad(theta))
                Rx = geometry.rotation_from_axis_angle(np.array([1, 0, 0]), np.deg2rad(180))
                R = np.matmul(Rz, Rx)
                q = geometry.quaternion_from_rotation(R)
                # compile pose
                tf = AutolabTransform(
                    origin=AutolabReferenceFrame(
                        time=rospy.Time.now(),
                        type=AutolabReferenceFrame.ENTITY_TYPE_MAP_ORIGIN,
                        name="map",
                        robot=self.robot_hostname
                    ),
                    target=AutolabReferenceFrame(
                        time=rospy.Time.now(),
                        type=AutolabReferenceFrame.ENTITY_TYPE_GROUND_TAG,
                        name=target,
                        robot=self.robot_hostname
                    ),
                    is_fixed=True,
                    is_static=False,
                    transform=Transform(
                        translation=Vector3(x=position[0], y=position[1], z=0),
                        rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    )
                )
                # add tf
                tfs.append(tf)
            except (KeyError, AttributeError, ValueError):
                pass
        # ---
        return tfs


if __name__ == '__main__':
    node = DistributedTFNode()
    # spin forever
    rospy.spin()
