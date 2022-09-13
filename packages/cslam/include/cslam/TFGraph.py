import g2o
from threading import Semaphore
from networkx import OrderedMultiDiGraph

from cslam_app.utils.T2Profiler import T2Profiler
from .G2OPoseGraphOptimizer import G2OPoseGraphOptimizer
from .utils import TF
from tf import transformations as tr
import numpy as np


class TFGraph(OrderedMultiDiGraph):

    def __init__(self, *args, **kwargs):
        super(TFGraph, self).__init__(*args, **kwargs)
        # create lock
        self._lock = Semaphore(1)

    def add_node(self, name, **attr):
        if "pose" in attr and not isinstance(attr["pose"], TF):
            raise ValueError("When given, attribute `pose` must be of type `TF`.")
        if "fixed" in attr and not isinstance(attr["fixed"], bool):
            raise ValueError("When given, attribute `fixed` must be of type `bool`.")
        # arg combinations
        if "fixed" in attr and attr["fixed"] and ("pose" not in attr or attr["pose"] is None):
            raise ValueError("When `fixed=True`, the argument `pose` is mandatory.")
        # default values
        if "fixed" not in attr:
            attr["fixed"] = False
        # does the node exist?
        if not self.has_node(name):
            attr["optimized"] = False
        # ---
        with self._lock:
            # print(f'Adding node "{name}" w/ {attr}')
            super(TFGraph, self).add_node(name, **attr)

    def add_measurement(self, origin: str, target: str, time: float, measurement: TF,
                        information: np.array = np.eye(6), **attr):
        self.add_edge(origin, target,
                      time=time,
                      measurement=measurement,
                      information=information,
                      **attr)

    def add_edge(self, u, v, key=None, **attr):
        if "measurement" not in attr:
            raise ValueError(f"Missing attribute `measurement` for edge ({u}, {v}).")
        if "time" not in attr:
            raise ValueError(f"Missing attribute `time` for edge ({u}, {v}).")
        if not isinstance(attr["measurement"], TF):
            raise ValueError("Edge attribute `measurement` must be of type `TF`.")
        # ---
        with self._lock:
            # print(f'Adding edge "({u}, {v})" w/ {attr}')
            super(TFGraph, self).add_edge(u, v, **attr)

    def add_nodes_from(self, nodes_for_adding, **attr):
        raise NotImplementedError("`add_nodes_from` not supported on instance of type `TFGraph`.")

    def add_edges_from(self, ebunch_to_add, **attr):
        raise NotImplementedError("`add_edges_from` not supported on instance of type `TFGraph`.")

    def add_weighted_edges_from(self, ebunch_to_add, weight="weight", **attr):
        raise NotImplementedError("`add_weighted_edges_from` not supported on instance "
                                  "of type `TFGraph`.")

    def is_fixed(self, name, default: bool = None) -> bool:
        if name not in self:
            if default is None:
                raise KeyError(f"Node `{name}` not found.")
            return default
        return self[name].get('fixed', default)

    def has_neighbor_of_type(self, name, neighbor_type):
        if name not in self:
            return False
        for nname in self[name]:
            if self.nodes[nname]['type'] == neighbor_type:
                return True
        return False

    def get_pose(self, name):
        if name not in self:
            return None
        if 'pose' not in self.nodes[name]:
            return None
        return self.nodes[name]['pose']

    def get_nearest_node_in_time(self, msec, type, precision=500):
        closest = None
        closest_node_time = None
        dt_msec = precision
        for nname, ndata in self.nodes(data=True):
            if ndata['type'] == type:
                a = nname.split('/')
                node_time = int(a[2])
                if abs(node_time-msec) < dt_msec:
                    closest = nname
                    closest_node_time = node_time
                    dt_msec = abs(node_time-msec)
        return closest, closest_node_time

    def optimize(self, max_iterations=20):
        optimizer = G2OPoseGraphOptimizer()
        id_to_name = {}
        name_to_id = {}
        # populate optimizer
        with self._lock:
            # add vertices
            with T2Profiler.profile("python-to-g2o-add-node"):
                for nname, ndata in self.nodes.items():
                    # compute node ID
                    node_id = len(id_to_name)
                    id_to_name[node_id] = nname
                    name_to_id[nname] = node_id
                    # get node pose and other attributes
                    pose = g2o.Isometry3d(g2o.Quaternion(ndata["pose"].Q('wxyz')), ndata["pose"].t) \
                        if "pose" in ndata else None
                    if "pose" in ndata:
                        print(f"ADDING POSE FOR: {nname}\t {ndata['pose'].t}")
                    else:
                        print(f"NO POSE FOR: {nname}")
                    fixed = ndata["fixed"] if "fixed" in ndata else False
                    # add vertex
                    optimizer.add_vertex(node_id, pose=pose, fixed=fixed)
            # add edges
            with T2Profiler.profile("python-to-g2o-add-edge"):
                for u, v, edata in self.edges.data():
                    # get nodes IDs
                    iu = name_to_id[u]
                    iv = name_to_id[v]
                    # get edge measurement and other attributes
                    measurement = edata["measurement"]
                    # add edge
                    optimizer.add_edge([iu, iv], g2o.Isometry3d(measurement.T))

        # optimize
        with T2Profiler.profile("g2o-optimize"):
            optimizer.optimize(max_iterations=max_iterations)

        # update graph
        with self._lock:
            for nid, nname in id_to_name.items():
                npose = optimizer.get_pose(nid)
                T = tr.compose_matrix(
                    translate=npose.t,
                    angles=tr.euler_from_matrix(npose.R)
                )
                q = tr.quaternion_from_matrix(T)
                if "pose" not in self.nodes[nname] or self.nodes[nname]["pose"] is None:
                    self.nodes[nname]["pose"] = TF(t=npose.t, q=q)
                else:
                    self.nodes[nname]["pose"].t = npose.t
                    self.nodes[nname]["pose"].q = q
                self.nodes[nname]["optimized"] = True
