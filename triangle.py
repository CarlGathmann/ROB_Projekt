import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
from stl import mesh

from utils import is_inside, transform_point, create_triangles

filepath = 'meshes/Test ROB Proj v2.stl'

fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
stl_mesh = mesh.Mesh.from_file(filepath)
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(stl_mesh.vectors, color='lightgrey', edgecolor='black'))
scale = stl_mesh.points.flatten()
ax.auto_scale_xyz(scale, scale, scale)

EPSILON = 5  # Adjust this value to control the threshold for "near"
AREA = 40  # create a meshgrid with 40 points in each direction
AMOUNT = 30
SCALING = 1  # Adjust this value to control the length of the vectors
DISTANCE = 2  # Adjust this value to control the distance from the surface

print("Area: ", AREA, "Amount: ", AMOUNT, "Scaling: ", SCALING, "Distance: ", DISTANCE)


def main():
    triangles = create_triangles(stl_mesh)

    #  create a meshgrid
    x, y, z = np.linspace(-AREA, AREA, AMOUNT), np.linspace(-AREA, AREA, AMOUNT), np.linspace(-AREA, AREA, AMOUNT)

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

                    if is_inside(triangle, transformed_point) and abs(height) < DISTANCE:
                        u[i, j, k], v[i, j, k], w[i, j, k] = triangle.normal * SCALING

                        ''' WEIRD STUFF HAPPENS HERE '''

                        edge, d = get_edge_and_dist(triangle, transformed_point)
                        if d <= EPSILON:
                            for adj_triangle in triangle.adjacent_triangles:
                                adj_trig_edge, adj_trig_d = get_edge_and_dist(adj_triangle, transformed_point)
                                if np.array_equal(adj_trig_edge, edge):
                                    weight = d / EPSILON
                                    blended_normal = weight * triangle.normal + (1 - weight) * adj_triangle.normal
                                    u[i, j, k], v[i, j, k], w[i, j, k] = blended_normal * SCALING

                        ''' WEIRD STUFF ENDS HERE '''

    # Plot the vector field
    ax.quiver(xx, yy, zz, u, v, w)

    # Show the plot
    plt.show()


if __name__ == "__main__":
    main()
