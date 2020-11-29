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


@dataclasses.dataclass
class DuckietownMap:
    tile_size: float
    tiles: dict
    objects: dict

    @property
    def width(self):
        return self.tile_size * len(self.tiles[0])

    @property
    def height(self):
        return self.tile_size * len(self.tiles)

    def tags(self, origin: str = 'world') -> List[TFMeasurement]:
        measurements = []
        # iterate over the floor tags
        for _, obj in self.objects.items():
            if obj['kind'] == "floor_tag":
                tag_id = obj['tag']['~TagInstance']['tag_id']
                position = obj['pose']['~SE2Transform']['p']
                theta = 0
                if "theta_deg" in obj['pose']['~SE2Transform']:
                    theta = obj['pose']['~SE2Transform']['theta_deg']
                target = "tag/%d" % tag_id
                # get position
                t = np.array([position[0], position[1], 0.0])
                # get rotation
                Rz = geometry.rotation_from_axis_angle(np.array([0, 0, 1]), np.deg2rad(theta))
                Rx = geometry.rotation_from_axis_angle(np.array([1, 0, 0]), np.deg2rad(180))
                R = np.matmul(Rz, Rx)
                q = geometry.quaternion_from_rotation(R)
                # compile pose
                measurements.append(TFMeasurement(
                    origin=origin,
                    target=target,
                    tf=TF(
                        t=t,
                        q=q
                    )
                ))
        # ---
        return measurements


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


def load_map(map_filepath: str) -> DuckietownMap:
    with open(map_filepath, 'r') as fin:
        map_data = yaml.safe_load(fin)
        return DuckietownMap(
            tile_size=map_data['tile_size'],
            tiles=map_data['tiles'],
            objects=map_data['objects']
        )


def TransformStamped_to_TF(msg: TransformStamped, stamp: bool = True):
    t = msg.transform.translation
    q = msg.transform.rotation
    time_ms = -1 if not stamp else int(msg.header.stamp.to_sec() * 1000)
    return TF(
        t=np.array([t.x, t.y, t.z]),
        q=np.array([q.x, q.y, q.z, q.w]),
        time_ms=time_ms
    )
