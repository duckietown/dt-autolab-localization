import logging

from flask import Flask
from flask_cors import CORS


class CSLAMrestAPI(Flask):

    def __init__(self, debug=False):
        # create Flask App
        super(CSLAMrestAPI, self).__init__(__name__)
        # apply CORS settings
        CORS(self)
        # configure logging
        logging.getLogger('werkzeug').setLevel(logging.DEBUG if debug else logging.WARNING)
        # setup API
        self.setup()

    def setup(self):
        from .actions.version import blueprint as api_version_bp
        from .actions.experiment import blueprint as experiment_bp
        # register blueprints (/*)
        self.register_blueprint(api_version_bp)
        # register blueprints (/experiment/*)
        self.register_blueprint(experiment_bp)
