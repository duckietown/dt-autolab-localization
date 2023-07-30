from matplotlib.pyplot import pause
from ..include.cslam import TFGraph, TF
from ..include.cslam.utils import create_info_matrix

import numpy as np

from networkx.drawing.nx_pylab import draw_networkx_edge_labels, draw

# Create 2 ground truth landmarks (1D)
# These simulate measuring a static april tag (measurement of the ground tags fixed from the watchtowers)
ground_truth = np.array([1, 2,])

# List of 3 variance values
variance_values = [0.1, 0.5, 0.3]

# Create constraints to simulate having perfect measurements on the other axis and angles
constrained_axis = [0, 1, 1, 1, 1, 1]

# Create 2 measurements (1D)
measurements = {'0'+str(i+1) : ground_truth[i] + np.random.normal(0, variance_values[i]) for i in range(len(ground_truth))}

# Instantiate a Graph object
graph = TFGraph()

# Add the nodes
graph.add_node(name="0", pose=TF(t=np.array([0,0,0])), fixed=True)
for i in range(3):
    graph.add_node(name=str(i))

# Generate the relative measurement b/w node 1 (origin) and node 2 (target)
measurements["12"] = ground_truth[1] - ground_truth[0] + np.random.normal(0, variance_values[2],)

print(f"Generated measurements: {measurements}")


# Create 2 information matrices (1D)
information_matrices = {
    key : create_info_matrix(
        stdd_position=variance_values[i],
        stdd_orientation=1,
        constraints=[False,]+[True]*5
        )
        for i, key in enumerate(measurements)}

# print(f"Generated information matrices: {information_matrices}")

# Add the measurements to the graph
for key, measurement in measurements.items():
    measurement_transform = TF(t=np.array([measurement,0,0]))
    graph.add_measurement(
        origin=[*key][0],
        target=[*key][1],
        time=0,
        measurement=measurement_transform,
        information=information_matrices[key]
        )


# Visualize the graph
import matplotlib.pyplot as plt
positions = {name : (value, 0) for name, value in zip(graph.nodes, [0]+list(measurements.values())[0:2])}
edge_labels = {name : value for name,value in zip(graph.nodes, measurements)} 
print(positions)
draw(graph,pos=positions, with_labels=True, connectionstyle='arc3, rad = 0.3')#,edge_labels=edge_labels)
plt.show()