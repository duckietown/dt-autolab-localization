#!/usr/bin/env python3
import os
from collections import defaultdict

import tf
import networkx as nx
import matplotlib.image as pimage
import matplotlib.pyplot as plt
from autolab_msgs.msg import \
    AutolabReferenceFrame
from autolab_msgs.msg import AutolabTransform

from cslam import TimedLocalizationExperiment
from cslam_app import manager, logger

# constants
MAP_NAME = "TTIC_large_loop"
EXPERIMENT_DURATION = 12
PRECISION_MSECS = 500
TILE_SIZE = 0.595
MAP_WIDTH = TILE_SIZE * 4
MAP_HEIGHT = TILE_SIZE * 5


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


def nodelist(g, prefix: str):
    return [n for n in g if n.startswith(prefix)]


if __name__ == '__main__':
    # launch experiment manager
    manager.start("/autolab/tf", AutolabTransform)

    # create experiment
    experiment = TimedLocalizationExperiment(manager, EXPERIMENT_DURATION, PRECISION_MSECS)
    experiment.start()

    # join experiment
    logger.info(f'Waiting {EXPERIMENT_DURATION} seconds for observation to come in...')
    experiment.join()

    # stop the manager
    manager.stop()

    # wait for enough observations to come in
    logger.info(f'Experiment terminated. The graph has '
                f'{experiment.graph.number_of_nodes()} nodes and '
                f'{experiment.graph.number_of_edges()} edges.')
    # optimize
    logger.info('Optimizing...')
    experiment.optimize()
    logger.info('Done!')

    # show graph
    G = experiment.graph
    print(f'Nodes: {G.number_of_nodes()}')
    print(f'Edges: {G.number_of_edges()}')

    # pos = nx.spring_layout(G)
    pos = {}

    for nname, ndata in G.nodes.data():
        pos[nname] = ndata["pose"].t[:2]

    # print poses
    for nname, ndata in G.nodes.data():
        if ndata["type"] not in [AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG,
                                 AutolabReferenceFrame.TYPE_WATCHTOWER_CAMERA]:
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
