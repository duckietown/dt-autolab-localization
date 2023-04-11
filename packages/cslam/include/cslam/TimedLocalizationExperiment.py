from typing import List

from .LocalizationExperiment import LocalizationExperiment
from .experiments import \
    ExperimentsManagerAbs


class TimedLocalizationExperiment(LocalizationExperiment):

    def __init__(self, manager: ExperimentsManagerAbs, duration: int, trackables: List[int],
                 **kwargs):
        super().__init__(manager, duration, trackables, **kwargs)
