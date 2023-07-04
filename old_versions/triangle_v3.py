import numpy as np
from stl import mesh

from utils import Triangle

a = np.array([0, 0, 0])
b = np.array([10, 0, 0])
c = np.array([0, 12, 0])

p = np.array([0, 6, 0])

filepath = '../meshes/Test ROB Proj v2.stl'
stl_mesh = mesh.Mesh.from_file(filepath)

# barycentric coordinates
lamd = np.linalg.solve(np.array([[1, 1, 1], [0, b[0] - a[0], c[0] - a[0]], [0, b[1] - a[1], c[1] - a[1]]]),
                       np.array([1, p[0] - a[0], p[1] - a[1]]))

# create dict for points and edges from mesh
vertices = dict()
triangles = []

for vectors, normal in zip(stl_mesh.vectors, stl_mesh.normals):
    for vector in vectors:
        vector = tuple(vector)
        normal = tuple(normal)
        if vector not in vertices:
            vertices[vector] = {normal}
        else:
            vertices.get(vector).add(normal)

    triangles.append(Triangle(vectors[0], vectors[1], vectors[2], normal))


def get_normals_of_vertex(vertex):
    return list(vertices.get(tuple(vertex)))


def get_normals_of_edge(vertex_1, vertex_2):
    v1 = vertices.get(tuple(vertex_1))
    v2 = vertices.get(tuple(vertex_2))

    return list(v1.intersection(v2))


def test():
    for vectors in stl_mesh.vectors:
        print("EDGES:")
        print(len(get_normals_of_edge(vectors[0], vectors[1])))
        print(len(get_normals_of_edge(vectors[1], vectors[2])))
        print(len(get_normals_of_edge(vectors[2], vectors[0])))
        print()
        print("VERTICES:")
        print(len(get_normals_of_vertex(vectors[0])) == 3)
        print(len(get_normals_of_vertex(vectors[1])) == 3)
        print(len(get_normals_of_vertex(vectors[2])) == 3)
        print()

test()

