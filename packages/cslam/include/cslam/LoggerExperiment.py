import copy
import os
import signal
import subprocess
from typing import cast

from autolab_msgs.msg import AutolabReferenceFrame
from geometry_msgs.msg import Transform, Quaternion

from cslam.utils import Transform_to_TF
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


class LoggerExperiment(ExperimentAbs):

    def __init__(self, manager: ExperimentsManagerAbs, duration: int, *args, **kwargs):
        super().__init__(manager, duration)
        # check params
        if 'destination' not in kwargs:
            raise KeyError("Parameter `destination` is mandatory.")
        self.destination = kwargs['destination']
        self._logger = None

    def __callback__(self, msg, _):
        pass

    def __start__(self):
        from cslam_app.experiments_manager import ExperimentsManager
        manager = cast(ExperimentsManager, self.manager)
        env = copy.deepcopy(os.environ)
        env['LCM_DEFAULT_URL'] = manager.communication_group._url
        # launch logger
        self._logger = subprocess.Popen(
            ["lcm-logger", self.destination],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
            preexec_fn=os.setpgrp
        )

    def __stop__(self):
        # stop recording
        try:
            os.killpg(os.getpgid(self._logger.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

    def __postprocess__(self):
        pass

    def __results__(self):
        return {}
