import dataclasses
import json
from typing import Optional

import numpy as np

from geometry_msgs.msg import Transform, TransformStamped
from tf import transformations as tr

INFTY = 9999999


class TF:
    _t: np.ndarray
    _q: np.ndarray
    _T: Optional[np.ndarray] = None
    time_ms: int = -1

    def __init__(self, t: Optional[np.ndarray] = None, q: Optional[np.ndarray] = None,
                 _T: Optional[np.ndarray] = None, time_ms: int = -1):
        self._t = t if t is not None else np.array([0, 0, 0])
        self._q = q if q is not None else np.array([0, 0, 0, 1])
        self._T = _T if _T is not None else tr.compose_matrix(
            translate=self.t,
            angles=tr.euler_from_quaternion(self.q)
        )
        self.time_ms = time_ms

    def Q(self, order='xyzw'):
        idx = ['xyzw'.index(a) for a in order]
        return np.array([self.q[i] for i in idx])

    @property
    def t(self):
        return self._t

    @property
    def q(self):
        return self._q

    @property
    def T(self):
        return self._T

    @t.setter
    def t(self, value: np.ndarray):
        self._t = value
        self._T = tr.compose_matrix(
            translate=value,
            angles=tr.euler_from_quaternion(self.q)
        )

    @q.setter
    def q(self, value: np.ndarray):
        self._q = value
        self._T = tr.compose_matrix(
            translate=self._t,
            angles=tr.euler_from_quaternion(value)
        )

    def __str__(self):
        return str({
            "t": self._t,
            "q": self._q,
            "_T": self._T,
            "time_ms": self.time_ms
        })

    @staticmethod
    def from_T(T: np.ndarray) -> 'TF':
        _, _, angles, trans, _ = tr.decompose_matrix(T)
        return TF(
            t=trans,
            q=tr.quaternion_from_euler(*angles),
            _T=T
        )


@dataclasses.dataclass
class TFMeasurement:
    origin: str
    target: str
    tf: TF


def create_info_matrix(stdd_position, stdd_orientation, constraints=None):
    if constraints is None:
        constraints = [True] * 6
    # start from an identity matrix
    m = np.eye(6)
    # position
    for i in range(0, 3):
        m[i, i] = 0 if not constraints[i] else 1.0 / (stdd_position ** 2)
    # orientation
    for i in range(3, 6):
        m[i, i] = 0 if not constraints[i] else 1.0 / (stdd_orientation ** 2)
    # ---
    return m


def Transform_to_TF(msg: Transform):
    t = msg.translation
    q = msg.rotation
    return TF(
        t=np.array([t.x, t.y, t.z]),
        q=np.array([q.x, q.y, q.z, q.w])
    )


def TransformStamped_to_TF(msg: TransformStamped, stamp: bool = True):
    tf = Transform_to_TF(msg.transform)
    tf.time_ms = -1 if not stamp else int(msg.header.stamp.to_sec() * 1000)
    return tf
