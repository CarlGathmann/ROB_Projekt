import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
from stl import mesh

from utils import dist_to_edges, transform_point, create_triangles, get_normal_and_interpolation

filepath = 'meshes/ro'

fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
stl_mesh = mesh.Mesh.from_file(filepath)
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(stl_mesh.vectors, color='lightgrey', edgecolor='black'))
scale = stl_mesh.points.flatten()
ax.auto_scale_xyz(scale, scale, scale)

EPSILON = 35  # Adjust this value to control the threshold for "near"
AREA = 40  # create a meshgrid with 40 points in each direction
AMOUNT = 35
SCALING = 1.5  # Adjust this value to control the length of the vectors
DISTANCE = 1  # Adjust this value to control the distance from the surface

print("Area: ", AREA, "Amount: ", AMOUNT, "Scaling: ", SCALING, "Distance: ", DISTANCE)


def main():
    triangles = create_triangles(stl_mesh)

    #  create a meshgrid
    x, y, z = np.linspace(-5, 40, AMOUNT), np.linspace(-5, 20, AMOUNT), np.linspace(-5, 20, AMOUNT)

    # Create a meshgrid of x, y, z coordinates
    xx, yy, zz = np.meshgrid(x, y, z)

    # Initialize the vector fields for each triangle
    u, v, w = np.zeros_like(xx), np.zeros_like(yy), np.zeros_like(zz)

    for triangle in triangles:
        for i in range(xx.shape[0]):
            for j in range(xx.shape[1]):
                for k in range(xx.shape[2]):
                    point = np.array([xx[i, j, k], yy[i, j, k], zz[i, j, k]])

                    #  transform point
                    transformed_point = transform_point(triangle.transformation_matrix, point)
                    height = transformed_point[0]

                    d1, d2, d3 = dist_to_edges(triangle, transformed_point)
                    inside = all([d1 >= 0, d2 >= 0, d3 >= 0])
                    close_to_edge = any([0 <= d1 <= EPSILON, 0 <= d2 <= EPSILON, 0 <= d3 <= EPSILON])

                    if inside and 0 < height < DISTANCE:  # point is inside triangle and close to surface
                        if close_to_edge:
                            normal, d = get_normal_and_interpolation(d1, d2, d3, triangle, EPSILON)

                            d = sum(d) / len(d)  # TODO: dunno if this is correct

                            weight = d / EPSILON
                            blended_normal = weight * triangle.normal + (1 - weight) * normal
                            blended_normal = blended_normal/np.linalg.norm(blended_normal)
                            u[i, j, k], v[i, j, k], w[i, j, k] = blended_normal * SCALING

                        else:
                            u[i, j, k], v[i, j, k], w[i, j, k] = triangle.normal * SCALING

    # Plot the vector field
    ax.quiver(xx, yy, zz, u, v, w)

    # Show the plot
    plt.show()


if __name__ == "__main__":
    main()
