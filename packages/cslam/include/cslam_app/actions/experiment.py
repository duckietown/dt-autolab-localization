from flask import request, Blueprint

from autolab_msgs.msg import AutolabReferenceFrame

from cslam import TimedLocalizationExperiment, LoggerExperiment, ExperimentStatus
from cslam_app import manager
from cslam_app.utils import response_ok, response_error

blueprint = Blueprint('experiment', __name__)

TRACKABLES = [
    AutolabReferenceFrame.TYPE_DUCKIEBOT_FOOTPRINT,
    AutolabReferenceFrame.TYPE_WATCHTOWER_CAMERA
]
EXPERIMENT_TYPES = {
    'TimedLocalizationExperiment': TimedLocalizationExperiment,
    'LoggerExperiment': LoggerExperiment
}


@blueprint.route('/experiment/create')
def _experiment_create():
    """
    Creates a new experiment without starting it.
    """
    # get args
    exp_type = request.args.get('type', 'TimedLocalizationExperiment')
    duration = request.args.get('duration', None)
    precision_ms = request.args.get('precision_ms', None)
    # check args
    if duration is None:
        return response_error("Argument `duration` is missing.")
    if precision_ms is None:
        return response_error("Argument `precision_ms` is missing.")
    # make a copy of the args
    kwargs = dict()
    kwargs.update(request.args)
    del kwargs['duration']
    del kwargs['precision_ms']
    # parse args
    try:
        duration = int(duration)
        precision_ms = int(precision_ms)
    except ValueError as e:
        return response_error(str(e))
    # create experiment
    if exp_type not in EXPERIMENT_TYPES:
        return response_error(f"Experiment type `{exp_type}` not found.")
    exp_class = EXPERIMENT_TYPES[exp_type]
    try:
        exp = exp_class(manager, duration, precision_ms, TRACKABLES, **kwargs)
    except KeyError as e:
        return response_error(f"Error: {str(e)}")
    return response_ok({
        'id': exp.id
    })


@blueprint.route('/experiment/start/<string:experiment_id>')
def _experiment_start(experiment_id: str):
    """
    Starts an existing experiment.
    """
    # get experiment
    if not manager.has(experiment_id):
        return response_error(f'Experiment with ID `{experiment_id}` not found.')
    # start experiment
    exp = manager.get(experiment_id)
    exp.start()
    return response_ok({
        'id': exp.id
    })


@blueprint.route('/experiment/stop/<string:experiment_id>')
def _experiment_stop(experiment_id: str):
    """
    Stops an existing experiment.
    """
    # get experiment
    if not manager.has(experiment_id):
        return response_error(f'Experiment with ID `{experiment_id}` not found.')
    exp = manager.get(experiment_id)
    # check status
    if exp.status == ExperimentStatus.CREATED:
        return response_error('You cannot stop an experiment that is not `RUNNING`.')
    # stop experiment
    exp.stop(block=False)
    # ---
    return response_ok({
        'id': exp.id
    })


@blueprint.route('/experiment/status/<string:experiment_id>')
def _experiment_status(experiment_id: str):
    """
    Returns the status of an existing experiment.
    """
    # get experiment
    if not manager.has(experiment_id):
        return response_error(f'Experiment with ID `{experiment_id}` not found.')
    # get experiment
    exp = manager.get(experiment_id)
    # return experiments status
    return response_ok({
        'id': exp.id,
        'status': exp.status.name
    })


@blueprint.route('/experiment/results/<string:experiment_id>')
def _experiment_results(experiment_id: str):
    """
    Stops an existing experiment.
    """
    # get experiment
    if not manager.has(experiment_id):
        return response_error(f'Experiment with ID `{experiment_id}` not found.')
    # get experiment
    exp = manager.get(experiment_id)
    # check experiment status
    if exp.status != ExperimentStatus.FINISHED:
        return response_error(f'Experiment has not `FINISHED` yet. '
                              f'Use API endpoint `experiment/status/<ID>` to check its status.')
    # get experiments results
    res = exp.results()
    return response_ok({
        'id': exp.id,
        'results': res
    })
