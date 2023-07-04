import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from stl import mesh

from utils_v2 import create_triangles, point_triangle_distance, f

filepath = 'meshes/Test ROB Proj v2.stl'

fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
stl_mesh = mesh.Mesh.from_file(filepath)
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(stl_mesh.vectors, color='lightgrey', edgecolor='black'))
scale = stl_mesh.points.flatten()
ax.auto_scale_xyz(scale, scale, scale)

EPSILON = 35  # Adjust this value to control the threshold for "near"
AMOUNT = 20
SCALING = 1.5  # Adjust this value to control the length of the vectors
DISTANCE = 1  # Adjust this value to control the distance from the surface


def main():
    triangles = create_triangles(stl_mesh)

    #  create a meshgrid
    x, y, z = np.linspace(-2, 35, AMOUNT), np.linspace(-2, 20, AMOUNT), np.linspace(-2, 7, AMOUNT)

    # Create a meshgrid of x, y, z coordinates
    xx, yy, zz = np.meshgrid(x, y, z)

    # Initialize the vector fields for each triangle
    u, v, w = np.zeros_like(xx), np.zeros_like(yy), np.zeros_like(zz)

    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            for k in range(xx.shape[2]):
                p = np.array([xx[i, j, k], yy[i, j, k], zz[i, j, k]])
                vec = np.array([0, 0, 0], dtype=float)
                force = np.array([0, 0, 0], dtype=float)
                for triangle in triangles:

                    dist, point = point_triangle_distance(triangle, p)

                    force += f(dist)
                    vec += p - point

                vec = vec / np.linalg.norm(vec)
                u[i, j, k], v[i, j, k], w[i, j, k] = vec * force

    # Plot the vector field
    ax.quiver(xx, yy, zz, u, v, w)

    # Show the plot
    plt.show()


if __name__ == '__main__':
    main()
