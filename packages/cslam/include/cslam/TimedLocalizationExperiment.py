from collections import defaultdict
from typing import List, Dict
import numpy as np

from autolab_msgs.msg import AutolabReferenceFrame
from geometry_msgs.msg import Transform, Quaternion

from tf import transformations as tr

from cslam import TFGraph
from cslam.utils import Transform_to_TF, TF, INFTY
from .experiments import \
    ExperimentAbs, \
    ExperimentsManagerAbs

WORLD_RFRAME = Transform_to_TF(Transform(rotation=Quaternion(x=0, y=0, z=0, w=1)))
MOVABLE_FRAMES = [
    AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG,
    AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT
]
FIXED_FRAMES = [
    AutolabReferenceFrame.TYPE_MAP_ORIGIN,
    AutolabReferenceFrame.TYPE_WORLD
]


class TimedLocalizationExperiment(ExperimentAbs):

    def __init__(self, manager: ExperimentsManagerAbs, duration: int, trackables: List[int],
                 **kwargs):
        super().__init__(manager, duration)
        # check params
        if 'precision_ms' not in kwargs:
            raise KeyError("Parameter `precision_ms` is mandatory.")
        self._precision_ms = int(kwargs['precision_ms'])
        # store properties
        self._trackables = trackables
        # store fixed TFs
        self._fixed_tfs = {}
        # create graph
        self._graph = TFGraph()

    @property
    def precision_ms(self) -> int:
        return self._precision_ms

    @property
    def graph(self) -> TFGraph:
        return self._graph

    def __callback__(self, msg, _):
        tf = Transform_to_TF(msg.transform)

        # store ALL fixed TFs
        if msg.is_fixed:
            tf_key = (msg.origin.name, msg.target.name)
            self._fixed_tfs[tf_key] = msg

        # measurement type 1: a fixed (solid, rigid, not-observed) TF
        if msg.is_fixed and msg.origin.type in FIXED_FRAMES:
            # TODO: nodes are are always added with absolute pose (i.e., wrt /world frame)
            #       tags are normally defined wrt the /map_name frame, this will break when
            #       a map is not aligned with /world. If we always run localization wrt the
            #       map origin, this is not an issue, because /world would be a "local" world
            #       to this map only.
            # add nodes
            self._graph.add_node(
                msg.origin.name,
                pose=WORLD_RFRAME,
                fixed=True,
                **self._node_attrs(msg.origin)
            )
            self._graph.add_node(
                msg.target.name,
                pose=tf,
                fixed=True,
                **self._node_attrs(msg.target)
            )

        # measurement type 2: a static (solid, rigid, observed) TF
        if msg.is_static:
            # add nodes
            if not self._graph.has_node(msg.origin.name):
                # When adding the origin node, populate its initial pose based
                # on the pose of the target node if available
                if self._graph.has_node(msg.target.name):
                    tf_target = self._graph.get_pose(msg.target.name)
                    # There is no pose associated with target node
                    if tf_target:
                        # Set initial pose based on target pose and transform
                        T_origin = np.dot(tf_target.T(), tr.inverse_matrix(tf.T()))
                        tf_origin = TF.from_T(T_origin)

                        self._graph.add_node(msg.origin.name, pose=tf_origin,
                                             **self._node_attrs(msg.origin))
                    else:
                        self._graph.add_node(msg.origin.name, **self._node_attrs(msg.origin))
                else:
                    self._graph.add_node(msg.origin.name, **self._node_attrs(msg.origin))

            if not self._graph.has_node(msg.target.name):
                # When adding the target node, populate its initial pose based
                # on the pose of the origin node if available
                if self._graph.has_node(msg.origin.name):
                    tf_origin = self._graph.get_pose(msg.origin.name)
                    if tf_origin:
                        # Set initial pose based on origin pose and transform
                        T_target = np.dot(tf_origin.T(), tf.T())
                        tf_target = TF.from_T(T_target)

                        self._graph.add_node(msg.target.name, pose=tf_target,
                                             **self._node_attrs(msg.target))
                    else:
                        self._graph.add_node(msg.target.name, **self._node_attrs(msg.target))
                else:
                    self._graph.add_node(msg.target.name, **self._node_attrs(msg.target))

            # add observations
            self._graph.add_measurement(msg.origin.name, msg.target.name, tf)

        # measurement type 3: a dynamic TF
        if (not msg.is_static) and (not msg.is_fixed):
            # handle origin
            origin_node_name = msg.origin.name
            origin_time_ms = int(msg.origin.time.to_sec() * 1000)
            if msg.origin.type in MOVABLE_FRAMES:
                origin_node_name = f'{msg.origin.name}/{int(origin_time_ms // self._precision_ms)}'
            # handle target
            target_node_name = msg.target.name
            target_time_ms = int(msg.target.time.to_sec() * 1000)
            if msg.target.type in MOVABLE_FRAMES:
                target_node_name = f'{msg.target.name}/{int(target_time_ms // self._precision_ms)}'

            # Add origin_node_name node if it doesn't exist
            if not self._graph.has_node(origin_node_name):

                # When adding the origin node, populate its initial pose based
                # on the pose of the target node if available
                if self._graph.has_node(target_node_name):
                    tf_target = self._graph.get_pose(target_node_name)
                    if tf_target:
                        # Set initial pose based on target pose and transform
                        T_origin = np.dot(tf_target.T(), tr.inverse_matrix(tf.T()))
                        tf_origin = TF.from_T(T_origin)

                        self._graph.add_node(origin_node_name, pose=tf_origin,
                                             **self._node_attrs(msg.origin))
                    else:
                        self._graph.add_node(origin_node_name, **self._node_attrs(msg.origin))
                else:
                    self._graph.add_node(origin_node_name, pose=TF(),
                                         **self._node_attrs(msg.origin))

            both_movable = msg.origin.type in MOVABLE_FRAMES and msg.target.type in MOVABLE_FRAMES
            if not both_movable:
                # Add target_node_name node if it doesn't exist
                # multiple observations from or of a static frame

                # Replace the time in target_node_name with that of the
                # Duckiebot deadreckon node nearest in time
                target_time = int(target_time_ms // self._precision_ms)
                closest, msec = self.graph.get_nearest_node_in_time (target_time, AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT, self._precision_ms)
                if closest:
                    a = target_node_name.split('/')
                    target_node_name = a[0] + '/' + a[1] + '/' + str(msec)

                if not self._graph.has_node(target_node_name):
                    # When adding the target node, populate its initial pose
                    # based on the pose of the origin node if available
                    tf_origin = self._graph.get_pose(origin_node_name)
                    if tf_origin:
                        T_target = np.dot(tf_origin.T(), tf.T())
                        tf_target = TF.from_T(T_target)

                        self._graph.add_node(target_node_name, pose=tf_target,
                                             **self._node_attrs(msg.target))
                    else:
                        self._graph.add_node(target_node_name, **self._node_attrs(msg.target))

                self._graph.add_measurement(origin_node_name, target_node_name,
                                            tf)
            else:
                self._graph.nodes[origin_node_name]['__tfs__'][
                    (msg.origin.name, msg.target.name)].append(msg)
                # TODO: this should be inside the message
                odom_info_mat = np.eye(6)
                odom_info_mat[3, 3] = INFTY
                odom_info_mat[4, 4] = INFTY
                # sequence of observations from and to movable frames
                if origin_node_name != target_node_name:
                    # get list of observations to combine
                    msgs = self._graph.nodes[origin_node_name]['__tfs__'][
                        (msg.origin.name, msg.target.name)]
                    # start with identity T
                    T = tr.compose_matrix()
                    for _msg in sorted(msgs, key=lambda _m: _m.origin.time.to_sec(), reverse=True):
                        _tf = Transform_to_TF(_msg.transform)
                        T = np.dot(_tf.T(), T)

                    # Add target_node_name node if it doesn't exist
                    if not self._graph.has_node(target_node_name):
                        tf_origin = self._graph.get_pose(origin_node_name)
                        if tf_origin:
                            T_target = np.dot(tf_origin.T(), T)
                            tf_target = TF.from_T(T_target)
                            self._graph.add_node(target_node_name, pose=tf_target,
                                                 **self._node_attrs(msg.target))
                        else:
                            self._graph.add_node(target_node_name, **self._node_attrs(msg.target))

                    self._graph.add_measurement(origin_node_name, target_node_name, TF.from_T(T), information=odom_info_mat)

    def __postprocess__(self):
        self.optimize()

    def __results__(self):
        trackable_frames = {
            ndata["__name__"] for _, ndata in self._graph.nodes(data=True)
            if ndata["type"] in self._trackables
        }
        return {nname: self.trajectory(nname) for nname in trackable_frames}

    def optimize(self):
        self._extend_graph()
        self._graph.optimize()

    def trajectory(self, node: str) -> List[Dict[str, List]]:
        # collect all timed nodes corresponding to the node to track
        traj = []
        for nname, ndata in self._graph.nodes(data=True):
            if ndata['__name__'] == node:
                # Filter out Duckiebot nodes that are not connected to
                # another Duckiebot node
                dbot_type = AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT
                if ndata["type"] == dbot_type:
                    if not self._graph.has_neighbor_of_type(nname, dbot_type):
                        continue

                T = tr.compose_matrix(
                    translate=ndata["pose"].t,
                    angles=tr.euler_from_quaternion(ndata["pose"].q)
                )
                traj.append({
                    'timestamp': ndata["time"],
                    'pose': T.tolist(),
                    # TODO: this is a hack, should be done properly
                    'observers': list({
                        p[0] for e in self._graph.out_edges(nname) for p in self._graph.in_edges(e[1])
                    } if 'footprint' in nname else {})
                })
        # sort trajectory by time
        strategy = lambda tf: tf['timestamp']
        traj = sorted(traj, key=strategy)
        # ---
        return traj

    def _extend_graph(self):
        # append fixed TFs to non-trackable leaf nodes
        untrackable_leaf_nodes = [
            (nname, ndata) for nname, ndata in self.graph.nodes(data=True)
            if self.graph.out_degree(nname) == 0 and ndata['type'] not in self._trackables
        ]
        # store new nodes/edges
        new_nodes = {}
        new_edges = {}
        # iterate over the leaf nodes and append fixed TFs
        for leaf_timed_name, leaf in untrackable_leaf_nodes:
            leaf_frame_name = leaf['__name__']
            # search for edges (leaf, X), or (X, leaf)
            for (origin, target), msg in self._fixed_tfs.items():
                if origin == leaf_frame_name:
                    # found an edge of type (leaf, X)
                    if msg.target.type not in self._trackables:
                        continue
                    # ---
                    new_node = f"{target}/{int((leaf['time'] * 1000) // self._precision_ms)}"
                    # HACK: Don't add nodes for trackable objects.
                    #       They should already exist
                    if not self._graph.has_node(new_node):
                        continue
                    #print(f'Origin: Adding node `{new_node}` and edge `{leaf_timed_name}` -> `{new_node}`')
                    # create new node
                    new_node_attrs = self._node_attrs(msg.target)
                    new_node_attrs['time'] = leaf['time']
                    new_nodes[new_node] = new_node_attrs
                    # create new edge
                    tf = Transform_to_TF(msg.transform)
                    new_edges[(leaf_timed_name, new_node)] = tf
                if target == leaf_frame_name:
                    # found an edge of type (X, leaf)
                    if msg.origin.type not in self._trackables:
                        continue
                    # ---
                    new_node = f"{origin}/{int((leaf['time'] * 1000) // self._precision_ms)}"
                    # HACK: Don't add nodes for trackable objects.
                    #       They should already exist
                    if not self._graph.has_node(new_node):
                        continue
                    #print(f'Target: Adding node `{new_node}` and edge `{new_node}` -> `{leaf_timed_name}`')
                    new_node_attrs = self._node_attrs(msg.origin)
                    new_node_attrs['time'] = leaf['time']
                    new_nodes[new_node] = new_node_attrs
                    # create new edge
                    tf = Transform_to_TF(msg.transform)
                    new_edges[(new_node, leaf_timed_name)] = tf
        # add new nodes
        for nname, ndata in new_nodes.items():
            if not self._graph.has_node(nname):
                self._graph.add_node(nname, **ndata)
        # add new edges
        for (origin, target), tf in new_edges.items():
            if not self._graph.has_edge(origin, target):
                self._graph.add_measurement(origin, target, tf, information=np.eye(6))  # * INFTY)

    @staticmethod
    def _node_attrs(rframe: AutolabReferenceFrame) -> dict:
        return {
            "time": rframe.time.to_sec(),
            "type": rframe.type,
            "robot": rframe.robot,
            "__name__": rframe.name,
            "__tfs__": defaultdict(list)
        }
