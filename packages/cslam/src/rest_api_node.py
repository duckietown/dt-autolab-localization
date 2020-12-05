#!/usr/bin/env python3

import sys

from cslam_app import manager, CSLAMrestAPI
from cslam_app.constants import CSLAM_API_PORT

from dt_class_utils import DTProcess, AppStatus

from autolab_msgs.msg import AutolabTransform


class CSLAMApp(DTProcess):
    
    def __init__(self):
        super(CSLAMApp, self).__init__('CSLAM')
        self._api = CSLAMrestAPI(debug=self.is_debug)
        self.status = AppStatus.RUNNING
        # register shutdown callback
        self.register_shutdown_callback(_kill)
        # launch experiment manager
        manager.start("/autolab/tf", AutolabTransform)
        # serve HTTP requests over the REST API
        self._api.run(host='0.0.0.0', port=CSLAM_API_PORT)


def _kill():
    sys.exit(0)


if __name__ == '__main__':
    app = CSLAMApp()
