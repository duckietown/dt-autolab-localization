import logging
from typing import Any

from cslam.experiments import ExperimentsManagerAbs
from dt_communication_utils import DTCommunicationGroup


class ExperimentsManager(ExperimentsManagerAbs):

    def __init__(self):
        super().__init__()
        # placeholders
        self._group = None
        self._sub = None

    @property
    def communication_group(self):
        return self._group

    def start(self, topic: str, msg_type: Any):
        if self._group is not None:
            raise ValueError("You cannot launch an ExperimentsManager more than once.")
        # create communication group
        self._group = DTCommunicationGroup(topic, msg_type, loglevel=logging.DEBUG)
        # create subscribers
        self._sub = self._group.Subscriber(self._cb)

    def stop(self):
        if self._group is not None:
            self._group.shutdown()


manager = ExperimentsManager()

__all__ = [
    'manager',
    'ExperimentsManager'
]
