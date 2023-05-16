import numpy as np


def edge_function(start, end, point):
    case = (point[1] - start[1]) * (end[2] - start[2]) - (point[2] - start[2]) * (end[1] - start[1])
    return case <= 0


def is_inside(triangle, point):
    e1 = edge_function(triangle.transformed_a, triangle.transformed_b, point)
    e2 = edge_function(triangle.transformed_b, triangle.transformed_c, point)
    e3 = edge_function(triangle.transformed_c, triangle.transformed_a, point)

    return e1 and e2 and e3


def is_near_edge(triangle, point):
    for edge in triangle.edges:
        d = np.linalg.norm(np.cross(edge, edge[0] - point)) / np.linalg.norm(edge)
        return d, edge


def create_transformation_matrix(triangle):
    transformation_matrix = np.zeros((4, 4))
    rot_matrix = np.zeros((3, 3))
    translation = triangle.vertex_a

    v = triangle.vertex_b - triangle.vertex_a
    v = v / np.linalg.norm(v)

    helper = triangle.vertex_c - triangle.vertex_a
    helper = helper / np.linalg.norm(helper)

    q = np.cross(v, helper)
    q = q / np.linalg.norm(q)

    w = np.cross(v, q)
    w = w / np.linalg.norm(w)

    rot_matrix[0:3, 0] = q
    rot_matrix[0:3, 1] = w
    rot_matrix[0:3, 2] = v

    det = round(np.linalg.det(rot_matrix), 10)
    if det == -1:
        w *= -1
        rot_matrix[0:3, 0] = q
        rot_matrix[0:3, 1] = w
        rot_matrix[0:3, 2] = v

    transformation_matrix[0:3, 0:3] = rot_matrix
    transformation_matrix[0:3, 3] = translation
    transformation_matrix[3, 3] = 1

    return transformation_matrix


def transform_point(transformation_matrix, point):
    point_4d = np.zeros(4)
    point_4d[0:3] = point
    point_4d[3] = 1
    return np.matmul(np.linalg.inv(transformation_matrix), point_4d)[0:3]


class Triangle:
    def __init__(self, a, b, c, normal):
        self.vertex_a = a
        self.vertex_b = b
        self.vertex_c = c
        self.normal = normal / np.linalg.norm(normal)
        self.transformation_matrix = create_transformation_matrix(self)
        self.transformed_a = transform_point(self.transformation_matrix, self.vertex_a)
        self.transformed_b = transform_point(self.transformation_matrix, self.vertex_b)
        self.transformed_c = transform_point(self.transformation_matrix, self.vertex_c)
        self.edges = [self.transformed_a - self.transformed_b,
                        self.transformed_b - self.transformed_c,
                        self.transformed_c - self.transformed_a]
        self.adjacent_triangles = []

    def __str__(self):
        return "Triangle: " + str(self.vertex_a) + ", " + str(self.vertex_b) + ", " + str(self.vertex_c)

    ''' WEIRD STUFF HAPPENS HERE '''
    def find_adjacent_triangles(self, triangles):
        for triangle in triangles:
            if triangle != self and self.is_adjacent(triangle):
                self.adjacent_triangles.append(triangle)

    def is_adjacent(self, triangle):
        edges = [(self.vertex_b - self.vertex_a),
                 (self.vertex_c - self.vertex_b),
                 (self.vertex_a - self.vertex_c)]
        triangle_edges = [(triangle.vertex_b - triangle.vertex_a),
                          (triangle.vertex_c - triangle.vertex_b),
                          (triangle.vertex_a - triangle.vertex_c)]
        for edge in edges:
            for triangle_edge in triangle_edges:
                if np.array_equal(edge, triangle_edge):
                    return True

    ''' END OF WEIRD STUFF '''


def create_triangles(mesh):
    triangles = []

    for vectors, normal in zip(mesh.vectors, mesh.normals):
        triangles.append(Triangle(vectors[0], vectors[1], vectors[2], normal))

    # TODO: still some stuff to do here
    for triangle in triangles:
        triangle.find_adjacent_triangles(triangles)
    return triangles
