import yaml
import geometry
import dataclasses
import numpy as np
from typing import List

from geometry_msgs.msg import TransformStamped


@dataclasses.dataclass
class TF:
    t: np.ndarray = np.array([0, 0, 0])
    q: np.ndarray = np.array([0, 0, 0, 1])
    time_ms: int = -1


@dataclasses.dataclass
class TFMeasurement:
    origin: str
    target: str
    tf: TF


def TransformStamped_to_TF(msg: TransformStamped, stamp: bool = True):
    t = msg.transform.translation
    q = msg.transform.rotation
    time_ms = -1 if not stamp else int(msg.header.stamp.to_sec() * 1000)
    return TF(
        t=np.array([t.x, t.y, t.z]),
        q=np.array([q.x, q.y, q.z, q.w]),
        time_ms=time_ms
    )
