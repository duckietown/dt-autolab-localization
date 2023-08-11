#!/usr/bin/env python3
import os
import time
from collections import defaultdict
from functools import partial
from typing import List, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3

import tf
import networkx as nx
import matplotlib

from cslam_app.utils.T2Profiler import T2Profiler

#matplotlib.use('GTK3Agg')
#matplotlib.use('TkAgg')
matplotlib.use('Agg')
#only agg works when running on TTIClargeloop (something to do with headless terminal)

import matplotlib.pyplot as plt
import matplotlib.image as pimage
from autolab_msgs.msg import AutolabReferenceFrame, AutolabTransform

from cslam import TimedLocalizationExperiment
from cslam import OnlineLocalizationExperiment

from cslam_app import manager, logger
from dt_duckiematrix_protocols import Matrix

# constants
MAP_NAME = "TTIC_large_loop"
EXPERIMENT_DURATION = 12
PRECISION_MSECS = 100
TRACKABLES = [
    AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT
]
TILE_SIZE = 0.595
MAP_WIDTH = TILE_SIZE * 4
MAP_HEIGHT = TILE_SIZE * 5

VERBOSE = True
PROFILING = True
ROS_TF_PUBLISHER = False

# TODO: parse from environment variable
LOG_DIR = None
LOG_DIR = "LOG_FILE_DIR"

def marker(frame_type: str) -> str:
    markers = {
        "world": "P",
        "autobot": "o",
        "myrobot": "o",
        "tag/4": ".",
        "tag/3": "s",
        "watchtower": "h",
    }
    for prefix, mark in markers.items():
        if frame_type.startswith(prefix):
            return mark
    return "x"


def color(frame_type: str) -> str:
    colors = {
        "world": "black",
        "autobot": "cornflowerblue",
        "myrobot": "cornflowerblue",
        "tag/4": "slategrey",
        "tag/3": "red",
        "watchtower": "orange",
    }
    for prefix, mark in colors.items():
        if frame_type.startswith(prefix):
            return mark
    return "green"


def nodelist(g, prefix: str):
    return [n for n in g if n.lstrip('/').startswith(prefix)]


matrix = Matrix(
    #"localhost" #seems to only work when running computation locally, doesn't work when running on TTIClargeloop
    "192.168.1.34", #CHANGE TO CORRECT IP
    auto_commit=True
)
matrix_vehicle_name = "map_0/vehicle_0"
world_vehicle_name = "myrobot"
robot = matrix.robots.DB21M(matrix_vehicle_name, raw_pose=True)


#a temporary visualization of the online experiment that takes a picture each time it optimizes and combines into
#an animation/video
imgs = []
def update_temp_renderer(experiment: OnlineLocalizationExperiment):
    temp_g = experiment.graph



def update_renderer(experiment: OnlineLocalizationExperiment):
    # find poses
    poses: List[Tuple[float, dict]] = []
    for nname, ndata in experiment.nodes(lock=False):
        if ndata["type"] not in [AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT]:
            continue
        if ndata["robot"] != world_vehicle_name:
            continue
        if not ndata["optimized"]:
            continue
        xyz = ndata["pose"].t
        rpy = list(tf.transformations.euler_from_quaternion(ndata["pose"].q))
        poses.append((ndata["time"], {
            "x": xyz[0],
            "y": xyz[1],
            "yaw": rpy[2],
        }))
    # no poses?
    if len(poses) <= 0:
        return
    # find most recent optimized pose
    poses = sorted(poses, key=lambda _t: _t[0], reverse=True)
    _, pose = poses[0]
    # set pose on the viewer
    robot.pose.x = pose["x"]
    robot.pose.y = pose["y"]
    robot.pose.yaw = pose["yaw"]
    robot.pose.commit()


if __name__ == '__main__':
    if ROS_TF_PUBLISHER:
        rospy.init_node('cslam-single-experiment-debug')
        br = tf2_ros.TransformBroadcaster()

    if PROFILING:
        T2Profiler.enabled(True)

    if LOG_DIR is not None:
        import threading

        def my_function():
            # Perform some computation / task here
            print("Running in a separate thread...")

        # Create a new thread and pass the function as the target
        thread = threading.Thread(target=my_function)

        # Start the thread
        thread.start()

    # launch experiment manager
    manager.start("/autolab/tf", AutolabTransform)

    # create experiment
    ONLINE = False

    if ONLINE:
        experiment = OnlineLocalizationExperiment(
            manager,
            TRACKABLES,
            precision_ms=PRECISION_MSECS,
            verbose=VERBOSE
        )
        experiment.on_post_optimize(partial(update_renderer, experiment))
    else:
        experiment = TimedLocalizationExperiment(
            manager,
            EXPERIMENT_DURATION,
            TRACKABLES,
            precision_ms=PRECISION_MSECS,
            verbose=VERBOSE
        )
        logger.info(f'Waiting {EXPERIMENT_DURATION} seconds for observations to come in...')

    experiment.start()

    # join experiment
    try:
        experiment.join()
    except KeyboardInterrupt:
        pass

    # stop the manager
    manager.stop()

    # wait for enough observations to come in
    logger.info(f'Experiment terminated. The graph has '
                f'{experiment.graph.number_of_nodes()} nodes and '
                f'{experiment.graph.number_of_edges()} edges.')

    # show graph
    G = experiment.graph
    print(f'Nodes: {G.number_of_nodes()}')
    print(f'Edges: {G.number_of_edges()}')

    T2Profiler.print()

    #print('Is weakly connected:' + nx.is_weakly_connected(G))



    # pos = {}
    # for nname, ndata in G.nodes.data():
    #     pos[nname] = ndata["pose"].t[:2] + [0, 1]
    # for entity in ["world", "watchtower", "autobot", "tag/3"]:
    #     nx.draw_networkx_nodes(
    #         G,
    #         pos,
    #         nodelist=nodelist(G, entity),
    #         node_shape=marker(entity),
    #         node_color=color(entity),
    #         node_size=300
    #     )
    #
    # edges = set()
    # for edge in G.edges:
    #     edges.add((edge[0], edge[1]))
    # nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='blue')

    pos = {}
    for nname, ndata in G.nodes.data():
        pos[nname] = ndata["pose"].t[:2]

    # print poses
    for nname, ndata in G.nodes.data():
        if ndata["type"] not in [AutolabReferenceFrame.TYPE_GROUND_TAG]:
            continue
        a = list(tf.transformations.euler_from_quaternion(ndata["pose"].q))
        # print(f'Node[{nname}][{ndata["type"]}]:\n\t xyz: {ndata["pose"].t}\n\t rpy: {a}\n')

        if ROS_TF_PUBLISHER:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = nname
            p, q = ndata["pose"].t, ndata["pose"].q
            t.transform = Transform(
                translation=Vector3(p[0], p[1], p[2]),
                rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            )
            br.sendTransform(t)

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

    #print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")

    if not ONLINE: #CHANGE BACK
        # draw map
        png_filename = f"{MAP_NAME}.png"
        png_filepath = os.path.join(os.environ.get("DT_REPO_PATH"), "assets", "maps", png_filename)
        map_png = pimage.imread(png_filepath)
        
        
        #print("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC")

        plt.imshow(
            map_png,
            origin='upper',
            extent=[0, MAP_WIDTH, 0, MAP_HEIGHT]
        )
        #overlays the png of the map under the graph
        
        #print("DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD")

        for entity in ["world", "watchtower", "myrobot", "autobot", "tag/3"]:
            nx.draw_networkx_nodes(
                G,
                pos,
                nodelist=nodelist(G, entity),
                node_shape=marker(entity),
                node_color=color(entity),
                node_size=150
            )

        #print("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")

        edges = set()
        for edge in G.edges:
            edges.add((edge[0], edge[1]))
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='lime')
        #TODO: figure out why duckiebot shows up as an edge in the image (if you change edge color the spot
        #representing the duckiebot changes to the same color)

        #print("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")

        plt.xlim(0, MAP_WIDTH)
        plt.ylim(0, MAP_HEIGHT)
        plt.subplots_adjust(left=0, bottom=0, right=0.99, top=0.99)

        #plt.show()

        #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        
        plt.savefig('/data/temp.png')
        #because the computation is running on a headless device, cannot show image but have to save it instead
        #to access the saved image run "scp duckie@TTIClargeloop.local:/data/temp.png ./" in terminal

    # ---
    # rospy.signal_shutdown("done")
