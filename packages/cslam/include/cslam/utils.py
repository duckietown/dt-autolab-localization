import dataclasses
import numpy as np

from geometry_msgs.msg import Transform, TransformStamped
from tf import transformations as tr


@dataclasses.dataclass
class TF:
    t: np.ndarray = np.array([0, 0, 0])
    q: np.ndarray = np.array([0, 0, 0, 1])
    time_ms: int = -1

    def Q(self, order='xyzw'):
        idx = ['xyzw'.index(a) for a in order]
        return np.array([self.q[i] for i in idx])

    def T(self):
        return tr.compose_matrix(translate=self.t, angles=tr.euler_from_quaternion(self.q))

    @staticmethod
    def from_T(T: np.ndarray):
        _, _, angles, trans, _ = tr.decompose_matrix(T)
        return TF(t=trans, q=tr.quaternion_from_euler(*angles))


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
