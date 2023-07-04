import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import normalize

EPSILON = 0.05  # Adjust this value to control the threshold for "near"
SCALING = 0.01


def main():
    triangles = [
        Triangle(np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0.5, 0.5, 0.5])),
        Triangle(np.array([0, 0, 0]), np.array([0, 1, 0]), np.array([0.5, 0.5, 0.5])),
        Triangle(np.array([0, 1, 0]), np.array([1, 0, 0]), np.array([1, 1, 0])),
    ]

    color = "grey"

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x, y, z = np.linspace(-2, 2, 80), np.linspace(-2, 2, 80), np.linspace(-2, 2, 80)
    xx, yy, zz = np.meshgrid(x, y, z)

    u, v, w = np.zeros_like(xx), np.zeros_like(yy), np.zeros_like(zz)

    for triangle in triangles:
        ax.plot_trisurf([triangle.vertex_a[0], triangle.vertex_b[0], triangle.vertex_c[0]],
                        [triangle.vertex_a[1], triangle.vertex_b[1], triangle.vertex_c[1]],
                        [triangle.vertex_a[2], triangle.vertex_b[2], triangle.vertex_c[2]],
                        color=color, alpha=0.5)

        points = np.array([xx, yy, zz]).transpose(1, 2, 3, 0)
        mask = np.apply_along_axis(lambda point: is_inside(triangle, point) or is_near_edge(triangle, point), 3, points)

        for t in triangles:
            if t != triangle and share_edge(triangle, t):
                edge_mask = np.apply_along_axis(lambda point: is_near_edge(triangle, point), 3, points)
                interp_mask = mask & edge_mask
                interp_points = points[interp_mask]
                interp_normals = interpolate_normals(triangle, t, interp_points)
                u[interp_mask] = interp_normals[:, 0] * SCALING
                v[interp_mask] = interp_normals[:, 1] * SCALING
                w[interp_mask] = interp_normals[:, 2] * SCALING
                mask &= ~edge_mask

        n = triangle.normal
        dot_products = np.einsum('ijkl, l->ijk', points, n)
        u[mask] = (n[0] * SCALING) / dot_products[mask]
        v[mask] = (n[1] * SCALING) / dot_products[mask]
        w[mask] = (n[2] * SCALING) / dot_products[mask]

    ax.quiver(xx, yy, zz, u, v, w)

    ax.set_xlim([-0.5, 1.5])
    ax.set_ylim([-0.5, 1.5])
    ax.set_zlim([-0.5, 1.5])

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.show()


class Triangle:
    def __init__(self, a, b, c):
        self.vertex_a = a
        self.vertex_b = b
        self.vertex_c = c
        self.normal = np.cross(b - a, b - c)
        self.normal = normalize([self.normal])[0]
        if np.dot(self.normal, [0, 0, 1]) < 0:  # temporary fix
            self.normal = -self.normal


def normalize_vector(u, v, w):
    magnitude = np.sqrt(u ** 2 + v ** 2 + w ** 2)
    u_norm = np.where(magnitude > 0, u / magnitude, 0)
    v_norm = np.where(magnitude > 0, v / magnitude, 0)
    w_norm = np.where(magnitude > 0, w / magnitude, 0)

    return u_norm, v_norm, w_norm


def share_edge(triangle1, triangle2):
    shared_vertices = {tuple(triangle1.vertex_a), tuple(triangle1.vertex_b), tuple(triangle1.vertex_c)} & \
                      {tuple(triangle2.vertex_a), tuple(triangle2.vertex_b), tuple(triangle2.vertex_c)}
    return len(shared_vertices) == 2


def interpolate_normals(triangle1, triangle2, points):
    edge_center = (np.array(
        list({tuple(triangle1.vertex_a), tuple(triangle1.vertex_b), tuple(triangle1.vertex_c)} &
             {tuple(triangle2.vertex_a), tuple(triangle2.vertex_b), tuple(triangle2.vertex_c)}))[0] +
                   np.array(list({tuple(triangle1.vertex_a), tuple(triangle1.vertex_b), tuple(triangle1.vertex_c)} &
                                 {tuple(triangle2.vertex_a), tuple(triangle2.vertex_b), tuple(triangle2.vertex_c)}))[
                       1]) / 2

    edge_distance = np.linalg.norm(points - edge_center, axis=1)
    normalized_distance = edge_distance / np.max(edge_distance)

    interp_normals = (1 - normalized_distance[:, np.newaxis]) * triangle1.normal + normalized_distance[:, np.newaxis] * \
                     triangle2.normal
    return normalize(interp_normals)


def is_inside(triangle, point):
    v0 = triangle.vertex_c - triangle.vertex_a
    v1 = triangle.vertex_b - triangle.vertex_a
    v2 = point - triangle.vertex_a

    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)
    dotpn = np.dot(triangle.normal, point)

    inv_denom = 1 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom

    return (u >= 0) and (v >= 0) and (u + v <= 1) and (0.1 < dotpn < 0.5)


def is_near_edge(triangle, point):
    edges = [
        (triangle.vertex_a, triangle.vertex_b),
        (triangle.vertex_b, triangle.vertex_c),
        (triangle.vertex_c, triangle.vertex_a),
    ]

    for edge in edges:
        d = np.linalg.norm(np.cross(edge[1] - edge[0], edge[0] - point)) / np.linalg.norm(edge[1] - edge[0])
        if d <= EPSILON:
            return True

    return False


def is_near_vertex(triangle, point):
    vertices = [triangle.vertex_a, triangle.vertex_b, triangle.vertex_c]

    for vertex in vertices:
        if np.linalg.norm(point - vertex) <= EPSILON:
            return True

    return False


if __name__ == "__main__":
    main()
