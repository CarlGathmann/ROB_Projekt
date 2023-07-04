import numpy as np

vertices = dict()  # dict of normals for each values


class Vertex:
    def __init__(self, array):
        self.values = array

    def __hash__(self):
        return hash(self.values.tostring())

    def __eq__(self, other):
        return np.array_equal(self.values, other.values)

    def __sub__(self, other):
        return self.values - other.values

    def __repr__(self):
        return str(self.values)


def get_normals_of_vertex(vertex):
    return list(vertices.get(vertex))


def get_normals_of_edge(edge):
    v1 = vertices.get(edge[0])
    v2 = vertices.get(edge[1])

    return list(v1.intersection(v2))


def get_normal_and_interpolation(d1, d2, d3, triangle, epsilon):
    '''
    if d1 <= epsilon and d2 <= epsilon:
        master_normal = create_master_normal(get_normals_of_vertex(triangle.vertex_a))
        return master_normal, [d1, d2]
    elif d2 <= epsilon and d3 <= epsilon:
        master_normal = create_master_normal(get_normals_of_vertex(triangle.vertex_b))
        return master_normal, [d2, d3]
    elif d1 <= epsilon and d3 <= epsilon:
        master_normal = create_master_normal(get_normals_of_vertex(triangle.vertex_c))
        return master_normal, [d1, d3]
    '''
    master_normal = np.array([0, 0, 0], dtype=float)
    d = []
    if d1 <= epsilon:
        master_normal += create_master_normal(get_normals_of_edge(triangle.edges[0]))
        d.append(d1)
    if d2 <= epsilon:
        master_normal += create_master_normal(get_normals_of_edge(triangle.edges[1]))
        d.append(d2)
    if d3 <= epsilon:
        master_normal += create_master_normal(get_normals_of_edge(triangle.edges[2]))
        d.append(d3)
    return master_normal, d


def create_master_normal(normal_list):
    master_normal = np.sum(normal_list, axis=0)
    master_normal = master_normal / np.linalg.norm(master_normal)
    return master_normal


def edge_function(start, end, point):
    case = -((point[1] - start[1]) * (end[2] - start[2]) - (point[2] - start[2]) * (end[1] - start[1]))
    return case


def dist_to_edges(triangle, point):
    """returns the distance to each corresponding edge of the triangle, given a point."""
    d1 = edge_function(triangle.transformed_a, triangle.transformed_b, point)
    d2 = edge_function(triangle.transformed_b, triangle.transformed_c, point)
    d3 = edge_function(triangle.transformed_c, triangle.transformed_a, point)

    return d1, d2, d3


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
    transformation_matrix[0:3, 3] = translation.values
    transformation_matrix[3, 3] = 1

    return transformation_matrix


def transform_point(transformation_matrix, point):
    point_4d = np.zeros(4)
    point_4d[0:3] = point.values if type(point) == Vertex else point
    point_4d[3] = 1
    return np.matmul(np.linalg.inv(transformation_matrix), point_4d)[0:3]


class Triangle:
    def __init__(self, a, b, c, normal):
        self.vertex_a = Vertex(a)
        self.vertex_b = Vertex(b)
        self.vertex_c = Vertex(c)
        self.normal = normal / np.linalg.norm(normal)
        self.transformation_matrix = create_transformation_matrix(self)
        self.transformed_a = transform_point(self.transformation_matrix, self.vertex_a)
        self.transformed_b = transform_point(self.transformation_matrix, self.vertex_b)
        self.transformed_c = transform_point(self.transformation_matrix, self.vertex_c)
        self.edges = [
            (self.vertex_a, self.vertex_b),
            (self.vertex_b, self.vertex_c),
            (self.vertex_c, self.vertex_a),
        ]


def create_triangles(mesh):
    triangles = []

    for vectors, normal in zip(mesh.vectors, mesh.normals):
        for vector in vectors:
            vertex = Vertex(vector)
            normal = normal / np.linalg.norm(normal)
            n = tuple(normal)
            if vertex not in vertices:
                vertices[vertex] = {n}
            else:
                vertices.get(vertex).add(n)
        triangles.append(Triangle(vectors[0], vectors[1], vectors[2], normal))
    return triangles
