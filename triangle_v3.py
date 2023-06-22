import numpy as np
from stl import mesh

from utils import Triangle

a = np.array([0, 0, 0])
b = np.array([10, 0, 0])
c = np.array([0, 12, 0])

p = np.array([0, 6, 0])

filepath = 'meshes/Test ROB Proj v2.stl'
stl_mesh = mesh.Mesh.from_file(filepath)

# barycentric coordinates
lamd = np.linalg.solve(np.array([[1, 1, 1], [0, b[0] - a[0], c[0] - a[0]], [0, b[1] - a[1], c[1] - a[1]]]),
                       np.array([1, p[0] - a[0], p[1] - a[1]]))

# create dict for points and edges from mesh
vertices = dict()
edges = dict()
triangles = []

for vectors, normal in zip(stl_mesh.vectors, stl_mesh.normals):
    edge_list = [abs(vectors[0] - vectors[1]), abs(vectors[1] - vectors[2]), abs(vectors[2] - vectors[0])]
    for vector, edge in zip(vectors, edge_list):
        vector = tuple(vector)
        edge = tuple(edge)
        normal = tuple(normal)
        if vector not in vertices:
            vertices[vector] = {normal}
        else:
            vertices.get(vector).add(normal)

        if edge not in edges:
            edges[edge] = {normal}
        else:
            edges.get(edge).add(normal)

    triangles.append(Triangle(vectors[0], vectors[1], vectors[2], normal))

print(vertices)
print(edges)
print(len(edges[tuple(abs(triangles[0].vertex_a - triangles[0].vertex_b))]))
print(tuple(abs(triangles[0].vertex_a - triangles[0].vertex_b)))
