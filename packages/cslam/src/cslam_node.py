#!/usr/bin/env python3

import os
from collections import defaultdict
from threading import Semaphore

import matplotlib.image as pimage
import matplotlib.pyplot as plt
import networkx as nx
import rosbag
import tf
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from cslam import TFGraph
from cslam.utils import TransformStamped_to_TF
from cslam.utils import load_map, TF

# TODO: this is temp stuff
from cslam.__temporary import _is_wheel, _is_excluded, _is_duckiebot_tag

# constants
BAG_NAME = "2020-11-20-22-52-08"
MAP_NAME = "TTIC_large_loop"
PRECISION_MSECS = 500
TILE_TEETH_SIZE = 0.02


class PoseGraphOptimizerNode:

    def __init__(self):
        # super().__init__(
        #     "cslam_node",
        #     node_type=NodeType.LOCALIZATION
        # )
        # create graph
        self._graph = TFGraph()
        # add world origin
        self._graph.add_node("world", pose=TF(), fixed=True)
        # create static TFs holder
        self._static_tfs = {}
        self._static_tfs_lock = Semaphore(1)
        # create subscribers
        # self._tf_sub = rospy.Subscriber("/tf", TFMessage, self.cb_tf)
        # self._tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self.cb_tf_static)

    def cb_tf(self, msg):
        for transform in msg.transforms:
            origin = transform.header.frame_id
            target = transform.child_frame_id
            # add nodes
            if not self._graph.has_node(origin):
                self._graph.add_node(origin, fixed=False)
            # if not self._graph.has_node(target):
            #     self._graph.add_node(target)

            tf = TransformStamped_to_TF(transform)

            if _is_duckiebot_tag(msg):
                target = f'{target}/{int(tf.time_ms // PRECISION_MSECS)}'

            # add measurement
            self._graph.add_measurement(origin, target, tf)

            # self._graph.add_transform(transform)

            # t, q = transform.transform.translation, transform.transform.rotation
            # print(
            #     f"registering dynamic TF:\n"
            #     f"\t{transform.header.frame_id} -> {transform.child_frame_id}:\n"
            #     f"\tposition: {[t.x, t.y, t.z]}\n"
            #     f"\torientation: {tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])}\n"
            # )

    def cb_tf_static(self, msg):
        for transform in msg.transforms:
            origin = transform.header.frame_id
            target = transform.child_frame_id
            # add nodes
            if not self._graph.has_node(origin):
                self._graph.add_node(origin, fixed=True)
            if not self._graph.has_node(target):
                self._graph.add_node(target, pose=TransformStamped_to_TF(transform), fixed=True)
            # add measurement
            # TODO: this is wrong, ground tags should be `fixed`
            # self._graph.add_measurement(origin, target, )

            key = (transform.header.frame_id, transform.child_frame_id)

            # if key not in self._static_tfs:
            #     t, q = transform.transform.translation, transform.transform.rotation
            #     print(
            #         f"registering static TF:\n"
            #         f"\t{key[0]} -> {key[1]}:\n"
            #         f"\tposition: {[t.x, t.y, t.z]}\n"
            #         f"\torientation: {tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])}\n"
            #     )

            # store TF
            with self._static_tfs_lock:
                self._static_tfs[key] = transform

    def optimize(self):
        self._graph.optimize()

    def get_graph(self):
        return self._graph


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


# def nodesize(frame_type: str) -> Optional[int]:
#     sizes = {
#         "world": 50,
#         "tag/4": 50,
#         "tag/3": 50,
#         "watchtower": 50,
#         "autobot": 1
#     }
#     for prefix, size in sizes.items():
#         if frame_type.startswith(prefix):
#             return size
#     return None


def nodelist(G, prefix: str):
    return [n for n in G if n.startswith(prefix)]


if __name__ == '__main__':
    bag_filename = f"{BAG_NAME}.bag"
    bag_filepath = os.path.join(os.environ.get("DT_REPO_PATH"), "assets", "logs", bag_filename)
    # create node
    node = PoseGraphOptimizerNode()
    topic_to_cb = {
        "/tf": node.cb_tf,
        "/tf_static": node.cb_tf_static
    }
    # read map
    map_filename = f"{MAP_NAME}.yaml"
    map_filepath = os.path.join(os.environ.get("DT_REPO_PATH"), "assets", "maps", map_filename)
    dtmap = load_map(map_filepath)
    map_tags = dtmap.tags()
    for tag in map_tags:
        node.cb_tf_static(TFMessage(transforms=[
            TransformStamped(
                header=Header(frame_id=tag.origin),
                child_frame_id=tag.target,
                transform=Transform(
                    translation=Vector3(
                        x=tag.tf.t[0], y=tag.tf.t[1], z=tag.tf.t[2]
                    ),
                    rotation=Quaternion(
                        x=tag.tf.q[0], y=tag.tf.q[1], z=tag.tf.q[2], w=tag.tf.q[3]
                    )
                )
            )
        ]))
    # read from bag
    last_t = 0
    with rosbag.Bag(bag_filepath) as bag:
        for topic, msg, msg_time in bag.read_messages(topics=list(topic_to_cb.keys())):
            if _is_wheel(msg):
                continue
            if _is_excluded(msg):
                continue
            # if _is_duckiebot_tag(msg):
            #     continue
            # if _is_ground_tag(msg):
            #     continue
            # if not _is_of_interest(msg):
            #     continue
            # TODO: this is temporary, static TFs should be synced
            if topic == "/tf_static":
                continue
            # TODO: this is temporary, should go when the devices are time-synced
            for i in range(len(msg.transforms)):
                msg.transforms[i].header.stamp = msg_time
            # fake-publish message
            topic_to_cb[topic](msg)

            origin = msg.transforms[0].header.frame_id
            target = msg.transforms[0].child_frame_id
            t = msg.transforms[0].transform.translation
            q = msg.transforms[0].transform.rotation
            q = [q.x, q.y, q.z, q.w]

            if origin.startswith('watchtower01') and '310' in target:
                _t = [t.x, t.y, t.z]
                _a = list(tf.transformations.euler_from_quaternion(q))
                print(f'Observation[{origin} -> {target}]: \n\t xyz: {_t}\n\t rpw: {_a}\n')

            # break

    # optimize
    node.optimize()
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
        extent=[TILE_TEETH_SIZE, dtmap.width + TILE_TEETH_SIZE, 0, dtmap.height]
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

    plt.xlim(0, dtmap.width)
    plt.ylim(0, dtmap.height)
    plt.subplots_adjust(left=0, bottom=0, right=0.99, top=0.99)

    plt.show()
    # ---
    # rospy.signal_shutdown("done")
