from ..include.cslam import TFGraph, TF
from ..include.cslam.utils import create_info_matrix

import numpy as np

STD_MEASUREMENTS = 0.1

# Create 5 ground truth poses (1D)
# These simulate measuring a static april tag (mearuement of the ground tags fixed from the watchtowers)
ground_truth = np.array([0, 1, 2, 3, 4])

# Create 5 measurements (1D)
measurements = ground_truth + np.random.normal(0, STD_MEASUREMENTS, size=ground_truth.shape)

# Create constraints to simulate having perfect measurements on the other axis and angles
constrained_axis = np.array([0, 1, 1, 1, 1, 1])

# Create 5 information matrices (1D)
information_matrices = np.array([create_info_matrix(stdd_position=STD_MEASUREMENTS,constraints=constrained_axis) for _ in range(5)])

# Instantiate a Graph object
graph = TFGraph()

# Add the ground truth origin node
graph.add_node(name="origin", pose=TF.from_T(np.zeros(6)),)

# Add the ground truth nodes
for measurement in measurements:
    graph.add_measurement()