import timeit

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from stl import mesh

from utils_v2 import create_triangles, point_triangle_distance, f

filepath = 'meshes/test.stl'

fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
stl_mesh = mesh.Mesh.from_file(filepath)
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(stl_mesh.vectors, color='lightgrey', edgecolor='black'))
scale = stl_mesh.points.flatten()
ax.auto_scale_xyz(scale, scale, scale)

EPSILON = 35  # Adjust this value to control the threshold for "near"
AMOUNT = 25
SCALING = 2  # Adjust this value to control the length of the vectors
DISTANCE = 2  # Adjust this value to control the distance from the surface


def single_point(triangles):
    p = np.array([np.random.randint(0, 10), np.random.randint(0, 10), np.random.randint(0, 7)])
    vec = np.array([0, 0, 0], dtype=float)
    for triangle, normal in zip(triangles[0], triangles[1]):
        dist, pp0 = point_triangle_distance(triangle, p)
        _dir = p - pp0
        if _dir.dot(normal) >= 0:
            vec += (p - pp0) * f(dist)


def main():
    triangles = create_triangles(stl_mesh)
    vector_field(triangles)


def vector_field(triangles):
    #  create a meshgrid
    x, y, z = np.linspace(-2, 35, AMOUNT), np.linspace(-2, 20, AMOUNT), np.linspace(-4, 15, AMOUNT)
    # Create a meshgrid of x, y, z coordinates
    xx, yy, zz = np.meshgrid(x, y, z)
    # Initialize the vector fields for each triangle
    u, v, w = np.zeros_like(xx), np.zeros_like(yy), np.zeros_like(zz)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            for k in range(xx.shape[2]):
                p = np.array([xx[i, j, k], yy[i, j, k], zz[i, j, k]])
                vec = np.array([0, 0, 0], dtype=float)
                for triangle, normal in zip(triangles[0], triangles[1]):
                    dist, pp0 = point_triangle_distance(triangle, p)
                    _dir = p - pp0
                    if _dir.dot(normal) >= 0:
                        vec += (p - pp0) * f(dist)
                u[i, j, k], v[i, j, k], w[i, j, k] = vec
    # Plot the vector field
    ax.quiver(xx, yy, zz, u, v, w)
    # Show the plot
    plt.show()


if __name__ == '__main__':
    main()
