from threading import Thread

from flask import request, Blueprint

from cslam import TimedLocalizationExperiment, ExperimentStatus
from cslam_app import manager
from cslam_app.utils import response_ok, response_error

blueprint = Blueprint('experiment', __name__)


@blueprint.route('/experiment/create')
def _experiment_create():
    """
    Creates a new experiment without starting it.
    """
    # get args
    duration = request.args.get('duration', None)
    precision_ms = request.args.get('precision_ms', None)
    # check args
    if duration is None:
        return response_error("Argument `duration` is missing.")
    if precision_ms is None:
        return response_error("Argument `precision_ms` is missing.")
    # parse args
    try:
        duration = int(duration)
        precision_ms = int(precision_ms)
    except ValueError as e:
        return response_error(str(e))
    # create experiment
    exp = TimedLocalizationExperiment(manager, duration, precision_ms)
    return response_ok({
        'experiment_id': exp.id
    })


@blueprint.route('/experiment/start/<str:exp_id>')
def _experiment_start(exp_id: str):
    """
    Starts an existing experiment.
    """
    # get experiment
    if not manager.has(exp_id):
        return response_error(f'Experiment with ID `{exp_id}` not found.')
    # start experiment
    exp = manager.get(exp_id)
    exp.start()
    return response_ok({
        'experiment_id': exp.id
    })


@blueprint.route('/experiment/stop/<str:exp_id>')
def _experiment_stop(exp_id: str):
    """
    Stops an existing experiment.
    """
    # get experiment
    if not manager.has(exp_id):
        return response_error(f'Experiment with ID `{exp_id}` not found.')
    # stop experiment
    exp = manager.get(exp_id)
    exp.stop(block=False)
    # ---
    return response_ok({
        'experiment_id': exp.id
    })


@blueprint.route('/experiment/results/<str:exp_id>')
def _experiment_stop(exp_id: str):
    """
    Stops an existing experiment.
    """
    # get experiment
    if not manager.has(exp_id):
        return response_error(f'Experiment with ID `{exp_id}` not found.')
    # get experiment
    exp = manager.get(exp_id)
    # check experiment status
    if exp.status != ExperimentStatus.FINISHED:
        return response_error(f'Experiment has not `FINISHED` yet. '
                              f'Use API endpoint `experiment/status/<ID>` to check its status.')
    # get experiments results
    res = exp.results()
    return response_ok({
        'results': res
    })
