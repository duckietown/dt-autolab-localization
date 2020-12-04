import logging
import os

from flask import Flask
from flask_cors import CORS

from .actions.version import blueprint as api_version_bp
from .actions.experiment import blueprint as experiment_bp


class CSLAMrestAPI(Flask):

    def __init__(self, debug=False):
        # enable debug mode
        if debug:
            os.environ['FLASK_DEBUG'] = '1'
        # create Flask App
        super(CSLAMrestAPI, self).__init__(__name__)
        # register blueprints (/*)
        self.register_blueprint(api_version_bp)
        # register blueprints (/experiment/*)
        self.register_blueprint(experiment_bp)
        # apply CORS settings
        CORS(self)
        # configure logging
        logging.getLogger('werkzeug').setLevel(logging.DEBUG if debug else logging.WARNING)
