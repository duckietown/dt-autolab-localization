import time
from collections import defaultdict
from threading import Thread
from typing import List

from autolab_msgs.msg import AutolabReferenceFrame

from cslam_app.utils.T2Profiler import T2Profiler
from .LocalizationExperiment import LocalizationExperiment
from .experiments import \
    ExperimentsManagerAbs


class OnlineLocalizationExperiment(LocalizationExperiment):

    def __init__(self, manager: ExperimentsManagerAbs, trackables: List[int], **kwargs):
        super().__init__(manager, -1, trackables, **kwargs)
        verbose = kwargs['verbose'] if 'verbose' in kwargs else False
        self._optimizer_timer = Thread(target=self._optimer_cb, args=(verbose,), daemon=True)
        self._optimize_every_secs = 2.0
        self._max_watchtower_to_groundtag_observations = 20
        self._num_keep_optimized_nodes = 1

    def __start__(self):
        self._optimizer_timer.start()

    def _prune_watchtower_to_groundtag(self, verbose: bool = False):
        watchtowers_groundtags = defaultdict(lambda: set())
        # find all (watchtower, groundtag) -> {(idx, time),} connected by at least one edge
        for node_name, node_data in self._graph.nodes.data():
            if node_data["type"] == AutolabReferenceFrame.TYPE_GROUND_TAG:
                groundtag_name = node_name
                for watchtower_name, _, idx, data in self._graph.in_edges(groundtag_name, keys=True, data=True):
                    watchtowers_groundtags[(watchtower_name, groundtag_name)].add((idx, data["time"]))

        if verbose:
            for (wt, gt), edata in watchtowers_groundtags.items():
                print(f"Found {len(edata)} edges {wt} -> {gt}")

        watchtowers_groundtags = {
            k: v
            for k, v in watchtowers_groundtags.items()
            if len(v) > self._max_watchtower_to_groundtag_observations
        }

        for (wt, gt), edata in watchtowers_groundtags.items():
            edata = sorted(edata, key=lambda t: t[1], reverse=True)
            to_delete = edata[self._max_watchtower_to_groundtag_observations:]
            for idx, _ in to_delete:
                self._graph.remove_edge(wt, gt, idx)
            if verbose:
                print(f"Removed {len(to_delete)} edges {wt} -> {gt}")

    def _prune_duckiebot_trajectory(self, verbose: bool = False):
        footprints = defaultdict(lambda: set())
        # find all (footprint) -> {(node_key, time, optimized),}
        for node_name, node_data in self._graph.nodes.data():
            if node_data["type"] == AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT:
                robot_name = node_data["robot"]
                footprints[robot_name].add((node_name, node_data["time"], node_data["optimized"]))

        for robot_name, ndata in footprints.items():
            if verbose:
                print(f"Found {len(ndata)} footprint nodes for robot '{robot_name}'")

            ndata = sorted(ndata, key=lambda t: t[1], reverse=True)

            first_optimized_idx = None
            for i, (_, t, optimized) in enumerate(ndata):
                if optimized:
                    first_optimized_idx = i
                    break

            if first_optimized_idx is not None:
                to_delete = ndata[first_optimized_idx+self._num_keep_optimized_nodes:]
                for node_key, _, _ in to_delete:
                    self._graph.remove_node(node_key)
                if verbose:
                    print(f"Removed {len(to_delete)} footprint nodes for robot '{robot_name}'")

        tags_to_delete = defaultdict(lambda: set())
        # find all (vehicle_tag) -> {node_key,}
        for node_name, node_data in self._graph.nodes.data():
            if node_data["type"] == AutolabReferenceFrame.TYPE_DUCKIEBOT_TAG:
                tag_name = node_data["__name__"]
                has_footprint = False
                # find footprint for tag
                for node_name2 in self._graph.successors(node_name):
                    node_type2 = self._graph.nodes[node_name2]["type"]
                    if node_type2 == AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT:
                        has_footprint = True
                        break
                # ---
                if not has_footprint:
                    tags_to_delete[tag_name].add(node_name)

        for tag_name, ndata in tags_to_delete.items():
            for node_key in ndata:
                self._graph.remove_node(node_key)
            if verbose:
                print(f"Removed {len(ndata)} tag nodes without a footprint for tag '{tag_name}'")

    def _optimer_cb(self, verbose: bool = False):
        last_optimization_time = time.time()
        do_optimize = lambda: time.time() - last_optimization_time > self._optimize_every_secs
        while True:
            if not do_optimize():
                time.sleep(0.1)
                continue
            # ---
            with self._lock:
                prev_num_nodes = self._graph.number_of_nodes()
                prev_num_edges = self._graph.number_of_edges()

                with T2Profiler.profile("online-optimize"):
                    with T2Profiler.profile("extend-graph"):
                        self._extend_graph()

                    with T2Profiler.profile("prune-edges-watchtower-to-groundtag"):
                        self._prune_watchtower_to_groundtag(verbose=verbose)

                    with T2Profiler.profile("prune-nodes-duckiebot-trajectory"):
                        self._prune_duckiebot_trajectory(verbose=verbose)

                    self._graph.optimize()

                last_optimization_time = time.time()

                if verbose:
                    print(f"Graph:\n"
                          f"\tNodes:\t{prev_num_nodes} -> {self._graph.number_of_nodes()}\n"
                          f"\tEdges:\t{prev_num_edges} -> {self._graph.number_of_edges()}\n")
