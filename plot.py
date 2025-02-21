import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def create_ellipsoid_z_rotation(center, radii, yaw_angle):
    """
    Create an ellipsoid rotated around the Z-axis.

    :param center: (x0, y0, z0) - Center of the ellipsoid
    :param radii: (a, b, c) - Semi-axis lengths
    :param yaw_angle: Rotation around the Z-axis (in radians)
    :return: Rotated ellipsoid points (X_rot, Y_rot, Z)
    """
    x0, y0, z0 = center
    a, b, c = radii

    # Generate standard ellipsoid points
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 25)
    U, V = np.meshgrid(u, v)

    X = a * np.cos(U) * np.sin(V)
    Y = b * np.sin(U) * np.sin(V)
    Z = c * np.cos(V)

    # Rotation matrix around Z-axis
    cos_theta = np.cos(yaw_angle)
    sin_theta = np.sin(yaw_angle)
    Rz = np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])

    # Apply rotation
    points = np.array([X.flatten(), Y.flatten(), Z.flatten()])
    rotated_points = Rz @ points  # Matrix multiplication

    # Reshape to match original shape
    X_rot = rotated_points[0].reshape(X.shape) + x0
    Y_rot = rotated_points[1].reshape(Y.shape) + y0
    Z_rot = rotated_points[2].reshape(Z.shape) + z0
    print(X_rot)
    return X_rot, Y_rot, Z_rot

# Example Usage
center = (4.75, -2.0, 1)  # Center of the ellipsoid
radii = (0.25, 1, 0.25)  # Semi-axis lengths (a, b, c)
yaw_angle = np.pi/8  # 45-degree rotation around Z-axis

# Generate the rotated ellipsoid
X_rot, Y_rot, Z_rot = create_ellipsoid_z_rotation(center, radii, yaw_angle)

# Plot the ellipsoid
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X_rot, Y_rot, Z_rot, color='c', alpha=0.5, edgecolor='k')

ax.view_init(0, 180, 0)
# Labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
