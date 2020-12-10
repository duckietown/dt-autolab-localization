import abc
import time
import threading
import uuid
from enum import IntEnum
from typing import Union, Dict

MAX_EXPERIMENT_DURATION_SECS = 60


class ExperimentStatus(IntEnum):
    CREATED = 0
    RUNNING = 1
    STOPPED = 2
    POSTPROCESSING = 3
    FINISHED = 8
    ERROR = 9


class ExperimentsManagerAbs(abc.ABC):

    def __init__(self):
        # create experiments holder
        self._experiments: Dict[str, ExperimentAbs] = {}

    def _cb(self, msg, header):
        for experiment in self._experiments.values():
            if experiment.status == ExperimentStatus.RUNNING:
                experiment.step(msg, header)

    def add(self, experiment: 'ExperimentAbs'):
        if not isinstance(experiment, ExperimentAbs):
            raise ValueError("Argument `experiment` must of type `cslam.ExperimentAbs`, "
                             f"got `{type(experiment).__name__}` instead.")
        self._experiments[experiment.id] = experiment

    def remove(self, experiment: Union['ExperimentAbs', 'str']):
        experiment = self._resolve(experiment)
        # remove experiment
        del self._experiments[experiment.id]

    def has(self, experiment: Union['ExperimentAbs', 'str']) -> bool:
        try:
            self._resolve(experiment)
            return True
        except KeyError:
            return False

    def get(self, experiment: 'str') -> 'ExperimentAbs':
        return self._resolve(experiment)

    def _resolve(self, experiment: Union['ExperimentAbs', 'str']) -> 'ExperimentAbs':
        if not isinstance(experiment, (ExperimentAbs, str)):
            raise ValueError("Argument `experiment` must of type `cslam.ExperimentAbs` or `str`, "
                             f"got `{type(experiment).__name__}` instead.")
        # get the ID if an experiment object was passed
        if isinstance(experiment, ExperimentAbs):
            experiment = experiment.id
        # make sure we have this experiment
        if experiment not in self._experiments:
            raise KeyError(f"Experiment `{experiment}` not found.")
        # ---
        return self._experiments[experiment]


class ExperimentAbs(abc.ABC):

    def __init__(self, manager: ExperimentsManagerAbs,
                 duration: int = MAX_EXPERIMENT_DURATION_SECS, **kwargs):
        if not isinstance(manager, ExperimentsManagerAbs):
            raise ValueError("Argument `manager` must of type `cslam.ExperimentsManagerAbs`, "
                             f"got `{type(manager).__name__}` instead.")
        self._status = ExperimentStatus.CREATED
        self._manager = manager
        self._id = str(uuid.uuid4())
        self._duration = duration
        self._heart = threading.Thread(target=self._heartbeat)
        self._stime = None
        self._manager.add(self)

    @property
    def id(self) -> str:
        return self._id

    @property
    def duration(self) -> int:
        return self._duration

    @property
    def manager(self) -> ExperimentsManagerAbs:
        return self._manager

    @property
    def status(self) -> ExperimentStatus:
        return self._status

    @property
    def alive(self) -> bool:
        return self._status in [ExperimentStatus.CREATED, ExperimentStatus.RUNNING] and \
               (time.time() - self._stime < self._duration)

    def stop(self, block: bool = True):
        if self._status == ExperimentStatus.STOPPED:
            return
        if self._status != ExperimentStatus.RUNNING:
            raise ValueError('You cannot stop an experiment that is not `RUNNING`.')
        self.__stop__()
        self._status = ExperimentStatus.STOPPED
        self.post_process(block)

    def start(self):
        self._stime = time.time()
        self._heart.start()
        self.__start__()
        self._status = ExperimentStatus.RUNNING

    def join(self, timeout: int = None):
        if self.alive:
            self._heart.join(timeout)

    def step(self, msg, header):
        if self._status != ExperimentStatus.RUNNING:
            return
        self.__callback__(msg, header)

    def post_process(self, block: bool = True):
        if self._status != ExperimentStatus.STOPPED:
            raise ValueError('You cannot postprocess an experiment until it reaches '
                             'the status `STOPPED`.')
        self._status = ExperimentStatus.POSTPROCESSING

        def _postprocessor():
            self.__postprocess__()
            self._status = ExperimentStatus.FINISHED

        if block:
            _postprocessor()
        else:
            threading.Thread(target=_postprocessor).start()

    def results(self):
        if self._status != ExperimentStatus.FINISHED:
            raise ValueError('You cannot fetch the results of an experiment until it reaches '
                             'the status `FINISHED`.')
        return self.__results__()

    def __start__(self):
        pass

    def __stop__(self):
        pass

    @abc.abstractmethod
    def __callback__(self, msg, header):
        pass

    @abc.abstractmethod
    def __postprocess__(self):
        pass

    @abc.abstractmethod
    def __results__(self):
        pass

    def _heartbeat(self):
        while self.alive:
            time.sleep(1)
        self.stop()
