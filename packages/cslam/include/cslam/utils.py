import dataclasses
import json
from typing import Optional

import numpy as np

from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from tf import transformations as tr

INFTY = 9999999


class TF:
    """This class represents a transformation in 3D space.

    Attributes:
        _t (np.ndarray): _description_
        _q (np.ndarray): _description_
        _T (np.ndarray): _description_
        time_ms (int): _description_

    Args:
        t (Optional[np.ndarray], optional): _description_. Defaults to None.
        q (Optional[np.ndarray], optional): _description_. Defaults to None.
        _T (Optional[np.ndarray], optional): _description_. Defaults to None.
        time_ms (int, optional): _description_. Defaults to -1.

    Returns:
        _type_: _description_
    """
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
    
    def __mul__(self, other: 'TF') -> 'TF':
        """Implement left multiplication of two TF objects T_b->c * T_a->b = T_a->c

        Notation source: http://jamessjackson.com/lie_algebra_tutorial/04-transformations/

        Args:
            other (TF): T_a->b

        Returns:
            TF: T_a->c
        """
        return TF.from_T(np.dot(self.T,other.T))

IDENTITY_TF = TF([0, 0, 0], [0, 0, 0, 1])

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
    """This function converts a Transform message to a TF object that can be used for operations.

    Args:
        msg (Transform): message containig the transform

    Returns:
        TF: the corresponding TF object
    """

    t = msg.translation
    q = msg.rotation
    return TF(
        t=np.array([t.x, t.y, t.z]),
        q=np.array([q.x, q.y, q.z, q.w])
    )

def TF_to_Transform(msg: TF):
    t = msg.t
    q = msg.q
    return Transform(
        translation=Vector3(x=t[0], y=t[1], z=t[2]),
        rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    )

def TransformStamped_to_TF(msg: TransformStamped, stamp: bool = True):
    tf = Transform_to_TF(msg.transform)
    tf.time_ms = -1 if not stamp else int(msg.header.stamp.to_sec() * 1000)
    return tf

if __name__ == '__main__':
    print("Testing TF class")
    # Translation along x-axis
    translation_x = 1.0  # 1 meter

    # Rotation angle around z-axis (90 degrees)
    rotation_angle_z = np.deg2rad(90)  # Convert degrees to radians

    # Create a 3x3 rotation matrix for the rotation around z-axis
    rotation_matrix_z = np.array([
        [np.cos(rotation_angle_z), -np.sin(rotation_angle_z), 0],
        [np.sin(rotation_angle_z), np.cos(rotation_angle_z), 0],
        [0, 0, 1]
    ])

    # Create a 4x4 transformation matrix T_a->b
    T_a_to_b = np.eye(4)
    T_a_to_b[:3, :3] = rotation_matrix_z
    T_a_to_b[:3, 3] = [translation_x, 0, 0]
    
    T_b_to_a = np.linalg.inv(T_a_to_b)

    # Transform T_a_to_b to a quaternion and translation vector
    q_a_to_b = tr.quaternion_from_matrix(T_a_to_b)
    t_a_to_b = T_a_to_b[:3, 3]

    # Transform T_b_to_a to a quaternion and translation vector
    q_b_to_a = tr.quaternion_from_matrix(T_b_to_a)
    t_b_to_a = T_b_to_a[:3, 3]

    # Create a TF object from the quaternion and translation vector for both transformations
    tf_b_to_a = TF(t=t_b_to_a, q=q_b_to_a)
    tf_a_to_b = TF(t=t_a_to_b, q=q_a_to_b)
    
    assert ((tf_b_to_a*tf_a_to_b).T == T_b_to_a@T_a_to_b).all()
    print("TF class works correctly")