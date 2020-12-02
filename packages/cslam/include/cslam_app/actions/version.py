from flask import Blueprint

from cslam_app.constants import API_VERSION
from cslam_app.utils import response_ok


blueprint = Blueprint('version', __name__)


@blueprint.route('/version')
def _version():
    # return current API version
    return response_ok({'version': API_VERSION})
