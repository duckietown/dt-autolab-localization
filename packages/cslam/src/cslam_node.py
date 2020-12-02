#!/usr/bin/env python3

import tf
import os
import time
from collections import defaultdict
import networkx as nx

import matplotlib.image as pimage
import matplotlib.pyplot as plt

from cslam import TFGraph
from cslam.utils import Transform_to_TF

from dt_class_utils import DTProcess
from dt_communication_utils import DTCommunicationGroup
from autolab_msgs.msg import \
    AutolabTransform, \
    AutolabReferenceFrame

from geometry_msgs.msg import Transform, Quaternion

# constants
BAG_NAME = "2020-11-20-22-52-08"
MAP_NAME = "TTIC_large_loop"
PRECISION_MSECS = 500
TRACKABLE_FRAME_TYPES = [
    AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG,
    AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT
]
EXPERIMENT_DURATION = 12
TILE_SIZE = 0.599
MAP_WIDTH = TILE_SIZE * 4
MAP_HEIGHT = TILE_SIZE * 5
WORLD_RFRAME = Transform_to_TF(Transform(rotation=Quaternion(0, 0, 0, 1)))


class PoseGraphOptimizerNode(DTProcess):

    def __init__(self):
        super().__init__(name='PoseGraphOptimizerNode')
        # create graph
        self._graph = TFGraph()
        # create communication group
        self._group = DTCommunicationGroup("/autolab/tf", AutolabTransform)
        # create subscribers
        self._tf_pub = None

    def start(self):
        self._tf_pub = self._group.Subscriber(self.cb_tf)

    def stop(self):
        pub = self._tf_pub
        self._tf_pub = None
        # ----
        if pub is not None:
            pub.shutdown()

    def cb_tf(self, msg, _):
        if self._tf_pub is None:
            return

        # measurement type 1: a fixed (solid, rigid, not-observed) TF
        if msg.is_fixed and msg.origin.type == AutolabReferenceFrame.TYPE_MAP_ORIGIN:
            # add nodes
            self._graph.add_node(msg.origin.name, pose=WORLD_RFRAME, fixed=True, **self._node_attrs(msg.origin))
            self._graph.add_node(msg.target.name, pose=Transform_to_TF(msg.transform), fixed=True, **self._node_attrs(msg.target))

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
                origin_node_name = f'{msg.origin.name}/{int(origin_time_ms // PRECISION_MSECS)}'
            # handle target
            target_node_name = msg.target.name
            if msg.target.type in TRACKABLE_FRAME_TYPES:
                target_time_ms = int(msg.target.time.to_sec() * 1000)
                target_node_name = f'{msg.target.name}/{int(target_time_ms // PRECISION_MSECS)}'
            # add nodes
            if not self._graph.has_node(origin_node_name):
                self._graph.add_node(origin_node_name, **self._node_attrs(msg.origin))
            if not self._graph.has_node(target_node_name):
                self._graph.add_node(target_node_name, **self._node_attrs(msg.target))
            # add observations

            print(origin_node_name, target_node_name)

            tf = Transform_to_TF(msg.transform)
            self._graph.add_measurement(origin_node_name, target_node_name, tf)

    def optimize(self):
        self._graph.optimize()

    def get_graph(self):
        return self._graph

    @staticmethod
    def _node_attrs(rframe: AutolabReferenceFrame) -> dict:
        return {
            "time": rframe.time.to_sec(),
            "type": rframe.type,
            "robot": rframe.robot
        }

def marker(frame_type: str) -> str:
    markers = {
        "world": "P",
        "tag/4": "o",
        "tag/3": "s",
        "watchtower": "h",
        "autobot": "."
    }
    for prefix, mark in markers.items():
        if frame_type.startswith(prefix):
            return mark
    return "x"


def color(frame_type: str) -> str:
    colors = {
        "world": "black",
        "tag/4": "cornflowerblue",
        "tag/3": "red",
        "watchtower": "orange",
        "autobot": "slategrey"
    }
    for prefix, mark in colors.items():
        if frame_type.startswith(prefix):
            return mark
    return "green"


def nodelist(G, prefix: str):
    return [n for n in G if n.startswith(prefix)]


if __name__ == '__main__':
    # create node
    node = PoseGraphOptimizerNode()
    node.start()
    node.logger.info(f'Waiting {EXPERIMENT_DURATION} seconds for observation to come in...')
    # wait for enough observations to come in
    time.sleep(EXPERIMENT_DURATION)
    node.stop()
    time.sleep(2)
    node.logger.info(f'Node stopped. The graph has '
                     f'{node.get_graph().number_of_nodes()} nodes and '
                     f'{node.get_graph().number_of_edges()} edges.')
    # optimize
    node.logger.info('Optimizing...')
    node.optimize()
    node.logger.info('Done!')
    # show graph
    G = node.get_graph()
    print(f'Nodes: {G.number_of_nodes()}')
    print(f'Edges: {G.number_of_edges()}')

    # pos = nx.spring_layout(G)
    pos = {}

    for nname, ndata in G.nodes.data():
        pos[nname] = ndata["pose"].t[:2]

    # print poses
    for nname, ndata in G.nodes.data():
        if ndata["type"] not in [AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG, AutolabReferenceFrame.TYPE_WATCHTOWER_CAMERA]:
            continue
        a = list(tf.transformations.euler_from_quaternion(ndata["pose"].q))
        print(f'Node[{nname}]:\n\t xyz: {ndata["pose"].t}\n\t rpw: {a}\n')

    links = defaultdict(set)
    for u, v, _ in G.edges:
        links[v].add(u)

    # print('Edges:\n\t')
    # for tag, obss in links.items():
    #     print('\tTag {}:\n\t\t'.format(tag) + '\n\t\t'.join(obss))
    #     print()

    # ==> This block places the nodes according to time
    # pos = {
    #     node: np.array([
    #         node_attr['time_ms'], 1 if node.startswith('watchtower') else 0
    #     ]) for node, node_attr in G.nodes.items()
    # }
    # min_time = min([v[0] for v in pos.values()])
    # pos = {n: p - [min_time, 0] for n, p in pos.items()}
    # <== This block places the nodes according to time

    # draw map
    png_filename = f"{MAP_NAME}.png"
    png_filepath = os.path.join(os.environ.get("DT_REPO_PATH"), "assets", "maps", png_filename)
    map_png = pimage.imread(png_filepath)
    plt.imshow(
        map_png,
        origin='lower',
        extent=[0, MAP_WIDTH, 0, MAP_HEIGHT]
    )

    for entity in ["world", "watchtower", "autobot", "tag/3", "tag/4"]:
        nx.draw_networkx_nodes(
            G,
            pos,
            nodelist=nodelist(G, entity),
            node_shape=marker(entity),
            node_color=color(entity),
            node_size=300
        )

    edges = set()
    for edge in G.edges:
        edges.add((edge[0], edge[1]))
    nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='ivory')

    plt.xlim(0, MAP_WIDTH)
    plt.ylim(0, MAP_HEIGHT)
    plt.subplots_adjust(left=0, bottom=0, right=0.99, top=0.99)

    plt.show()
    # ---
    # rospy.signal_shutdown("done")
