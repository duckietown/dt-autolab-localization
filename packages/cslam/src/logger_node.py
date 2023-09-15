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

from autolab_msgs.msg import AutolabReferenceFrame, AutolabTransform

from cslam import TimedLocalizationExperiment
from cslam import OnlineLocalizationExperiment
from cslam import LoggerExperiment

from cslam_app import manager, logger
from dt_duckiematrix_protocols import Matrix

# constants
MAP_NAME = "TTIC_large_loop"
EXPERIMENT_DURATION = 22
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

if __name__ == '__main__':

    # launch experiment manager
    manager.start("/autolab/tf", AutolabTransform)

    experiment = LoggerExperiment(
            manager,
            duration=EXPERIMENT_DURATION,
            trackables=TRACKABLES,
            precision_ms=PRECISION_MSECS,
            verbose=VERBOSE,
            destination="/data/log"
        )
    
    experiment.start()

    # join experiment
    try:
        experiment.join()
    except KeyboardInterrupt:
        pass

    # stop the manager
    manager.stop()