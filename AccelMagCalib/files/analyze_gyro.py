import pandas as pd
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

pd.set_option('display.precision', 15)
df = pd.read_csv("gyro2.csv", header=None, names=["x", "y", "z"])
arr = df.to_numpy(dtype=float)

import numpy as np

def fit_3d_line_and_distances(points):
    """
    Fits a 3D line to a set of points using PCA and calculates distances.

    Args:
        points (np.ndarray): An array of shape (N, 3) representing N 3D points.

    Returns:
        tuple: (point_on_line, line_direction, distances)
    """
    # Calculate the centroid (mean) of the points, which is a point on the line
    centroid = np.mean(points, axis=0)

    # Center the points relative to the centroid
    points_centered = points - centroid

    # Calculate the covariance matrix
    covariance_matrix = np.cov(points_centered, rowvar=False)

    # Perform Eigenvalue Decomposition
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)

    # The direction of the best-fit line is the eigenvector corresponding 
    # to the largest eigenvalue (first principal component).
    line_direction = eigenvectors[:, np.argmax(eigenvalues)]

    # Normalize the direction vector
    line_direction = line_direction / np.linalg.norm(line_direction)

    # Calculate the distance of each point to the fitted line
    # Distance d = |(P - A) x v| / |v|, where A is a point on the line (centroid), 
    # P is the data point, and v is the unit direction vector.
    vector_to_point = points - centroid
    cross_product = np.cross(vector_to_point, line_direction)
    distances = np.linalg.norm(cross_product, axis=1) / np.linalg.norm(line_direction) # line_direction is already normalized, so divide by 1.

    return centroid, line_direction, distances

# Example Usage:
# Generate some sample 3D points
# points data structure: [[x1, y1, z1], [x2, y2, z2], ...]
df["x"].plot()
plt.show()
df["y"].plot()
plt.show()
df["z"].plot()
plt.show()
# point_on_line, direction_vector, point_distances = fit_3d_line_and_distances(arr)
# print("done")
# kmeans = KMeans(n_clusters=3, init='k-means++', random_state=0, n_init=10)

# 3. Fit the model to your data
# This step runs the K-Means algorithm.
print(df.mean(axis=0))
# data_2d = point_distances.reshape(-1, 1)
# kmeans.fit(data_2d)

# # 4. Get the results
# # The 'labels_' attribute contains the cluster assignment (0 or 1) for each input data point.
# clusters = kmeans.predict(data_2d)
# centroids = kmeans.cluster_centers_

# # The 'cluster_centers_' attribute contains the coordinates of the 2 final centroids.
# centroids = kmeans.cluster_centers_



# plt.figure(figsize=(8, 4))
# plt.scatter(point_distances, np.zeros_like(point_distances), c=clusters, cmap='viridis', marker='o')
# plt.scatter(centroids.flatten(), np.zeros_like(centroids.flatten()), color='red', marker='x', s=100, label='Centroids')
# plt.title('K-Means Clustering with 2 Centroids (1D Data)')
# plt.xlabel('Data Value')
# plt.yticks([]) # Hide y-axis ticks for 1D visualization
# plt.legend()
# plt.show()


# print(f"Point on the line (centroid): {point_on_line}")
# print(f"Line direction vector: {direction_vector}")
# print(f"Distances of points to the line: {point_distances}")
