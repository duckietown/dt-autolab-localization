import g2o
import geometry
from threading import Semaphore
from networkx import OrderedMultiDiGraph
from .G2OPoseGraphOptimizer import G2OPoseGraphOptimizer
from .utils import TF
from tf import transformations


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
        # ---
        with self._lock:
            print(f'Adding node "{name}" w/ {attr}')
            super(TFGraph, self).add_node(name, **attr)

    def add_measurement(self, origin: str, target: str, measurement: TF):
        self.add_edge(origin, target, measurement=measurement)

    def add_edge(self, u, v, key=None, **attr):
        if "measurement" not in attr:
            raise ValueError(f"Missing attribute `measurement` for edge ({u}, {v}).")
        if not isinstance(attr["measurement"], TF):
            raise ValueError("Edge attribute `measurement` must be of type `TF`.")
        # ---
        with self._lock:
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

    def optimize(self, max_iterations=20):
        optimizer = G2OPoseGraphOptimizer()
        id_to_name = {}
        name_to_id = {}
        # populate optimizer
        with self._lock:
            # add vertices
            for nname, ndata in self.nodes.items():
                # compute node ID
                node_id = len(id_to_name)
                id_to_name[node_id] = nname
                name_to_id[nname] = node_id
                # get node pose and other attributes
                pose = g2o.Isometry3d(g2o.Quaternion(ndata["pose"].q), ndata["pose"].t) \
                    if "pose" in ndata else None
                fixed = ndata["fixed"] if "fixed" in ndata else False
                # add vertex
                optimizer.add_vertex(node_id, pose=pose, fixed=fixed)
            # add edges
            for u, v, edata in self.edges.data():
                # get nodes IDs
                iu = name_to_id[u]
                iv = name_to_id[v]
                # get edge measurement and other attributes
                measurement = edata["measurement"]
                T = transformations.compose_matrix(
                    translate=measurement.t,
                    angles=transformations.euler_from_quaternion(measurement.q)
                )
                # add edge
                optimizer.add_edge([iu, iv], g2o.Isometry3d(T))
        # optimize
        optimizer.optimize(max_iterations=max_iterations)
        # update graph
        with self._lock:
            for nid, nname in id_to_name.items():
                npose = optimizer.get_pose(nid)
                q = geometry.quaternion_from_rotation(npose.R)
                self.nodes[nname]["pose"] = TF(t=npose.t, q=q)
