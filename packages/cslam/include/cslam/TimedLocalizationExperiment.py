from typing import List

from autolab_msgs.msg import AutolabReferenceFrame
from geometry_msgs.msg import Transform, Quaternion

from cslam import TFGraph
from cslam.utils import Transform_to_TF, TF
from .experiments import \
    ExperimentAbs, \
    ExperimentsManagerAbs

WORLD_RFRAME = Transform_to_TF(Transform(rotation=Quaternion(0, 0, 0, 1)))
TRACKABLE_FRAME_TYPES = [
    AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG,
    AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT
]


class TimedLocalizationExperiment(ExperimentAbs):

    def __init__(self, manager: ExperimentsManagerAbs, duration: int, precision_ms: int):
        super().__init__(manager, duration)
        # store properties
        self._precision_ms = precision_ms
        # create graph
        self._graph = TFGraph()

    @property
    def precision_ms(self) -> int:
        return self._precision_ms

    @property
    def graph(self) -> TFGraph:
        return self._graph

    def __callback__(self, msg, _):
        # measurement type 1: a fixed (solid, rigid, not-observed) TF
        if msg.is_fixed and msg.origin.type == AutolabReferenceFrame.TYPE_MAP_ORIGIN:
            # add nodes
            self._graph.add_node(
                msg.origin.name,
                pose=WORLD_RFRAME,
                fixed=True,
                **self._node_attrs(msg.origin)
            )
            self._graph.add_node(
                msg.target.name,
                pose=Transform_to_TF(msg.transform),
                fixed=True,
                **self._node_attrs(msg.target)
            )

        # measurement type 2: a static (solid, rigid, observed) TF
        if msg.is_static:
            # add nodes
            if not self._graph.has_node(msg.origin.name):
                self._graph.add_node(msg.origin.name, **self._node_attrs(msg.origin))
            if not self._graph.has_node(msg.target.name):
                self._graph.add_node(msg.target.name, **self._node_attrs(msg.target))
            # add observations
            tf = Transform_to_TF(msg.transform)
            self._graph.add_measurement(msg.origin.name, msg.target.name, tf)

        # measurement type 3: a dynamic TF
        if (not msg.is_static) and (not msg.is_fixed):
            # handle origin
            origin_node_name = msg.origin.name
            if msg.origin.type in TRACKABLE_FRAME_TYPES:
                origin_time_ms = int(msg.origin.time.to_sec() * 1000)
                origin_node_name = f'{msg.origin.name}/{int(origin_time_ms // self._precision_ms)}'
            # handle target
            target_node_name = msg.target.name
            if msg.target.type in TRACKABLE_FRAME_TYPES:
                target_time_ms = int(msg.target.time.to_sec() * 1000)
                target_node_name = f'{msg.target.name}/{int(target_time_ms // self._precision_ms)}'

            print(origin_node_name, target_node_name)

            # add nodes
            if not self._graph.has_node(origin_node_name):
                self._graph.add_node(origin_node_name, **self._node_attrs(msg.origin))
            if not self._graph.has_node(target_node_name):
                self._graph.add_node(target_node_name, **self._node_attrs(msg.target))
            # add observations
            tf = Transform_to_TF(msg.transform)
            self._graph.add_measurement(origin_node_name, target_node_name, tf)

    def __postprocess__(self):
        self.optimize()

    def __results__(self):
        return {
            self.trajectory(nname) for nname, ndata in self._graph.nodes(data=True)
            if ndata["type"] in TRACKABLE_FRAME_TYPES
        }

    def optimize(self):
        self._graph.optimize()

    def trajectory(self, node: str) -> List[TF]:
        # collect all timed nodes corresponding to the node to track
        traj = []
        for _, ndata in self._graph.nodes(data=True):
            if ndata['__name__'] == node:
                traj.append(ndata["pose"])
        # sort trajectory by time
        strategy = lambda tf: tf if tf is None else tf.time_ms
        traj = sorted(traj, key=strategy)
        # ---
        return traj

    @staticmethod
    def _node_attrs(rframe: AutolabReferenceFrame) -> dict:
        return {
            "time": rframe.time.to_sec(),
            "type": rframe.type,
            "robot": rframe.robot,
            "__name__": rframe.name
        }
