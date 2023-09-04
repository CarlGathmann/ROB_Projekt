import numpy as np

FULL_FORCE = 0.1
NO_FORCE = 1
INTERVALL = NO_FORCE - FULL_FORCE


def create_triangles(mesh):
    triangles = []
    normals = []
    for vectors, normal in zip(mesh.vectors, mesh.normals):
        triangles.append(np.array([vectors[0], vectors[1], vectors[2]]))
        normals.append(normal)
    return triangles, normals


def influence(dist):
    if dist <= FULL_FORCE:
        return 5  # what ever is much
    elif FULL_FORCE < dist <= NO_FORCE:
        return -(1 / INTERVALL) * dist + NO_FORCE / INTERVALL
    else:
        return 0


def point_triangle_distance(tri, p):
    # vectors
    b_0 = tri[0]
    e_0 = tri[1] - b_0
    e_1 = tri[2] - b_0
    d_0 = b_0 - p

    # dot products
    a = np.dot(e_0, e_0)
    b = np.dot(e_0, e_1)
    c = np.dot(e_1, e_1)
    d = np.dot(e_0, d_0)
    e = np.dot(e_1, d_0)
    f = np.dot(d_0, d_0)

    det = a * c - b * b
    s = b * e - c * d
    t = b * d - a * e

    if s + t <= det:
        if s < 0:
            if t < 0:
                # region 4
                if d < 0:
                    t = 0
                    if -d >= a:
                        s = 1
                        sqr_distance = a + 2 * d + f
                    else:
                        s = -d / a
                        sqr_distance = d * s + f
                else:
                    s = 0
                    if e >= 0:
                        t = 0
                        sqr_distance = f
                    else:
                        if -e >= c:
                            t = 1
                            sqr_distance = c + 2 * e + f
                        else:
                            t = -e / c
                            sqr_distance = e * t + f
                # end of region 4
            else:
                # region 3
                s = 0
                if e >= 0:
                    t = 0
                    sqr_distance = f
                else:
                    if -e >= c:
                        t = 1
                        sqr_distance = c + 2 * e + f
                    else:
                        t = -e / c
                        sqr_distance = e * t + f
                # end of region 3
        else:
            if t < 0:
                # region 5
                t = 0
                if d >= 0:
                    s = 0
                    sqr_distance = f
                else:
                    if -d >= a:
                        s = 1
                        sqr_distance = a + 2 * d + f
                    else:
                        s = -d / a
                        sqr_distance = d * s + f
                # end of region 5
            else:
                # region 0
                inv_det = 1.0 / det
                s = s * inv_det
                t = t * inv_det
                sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f
    else:
        if s < 0:
            # region 2
            tmp0 = b + d
            tmp1 = c + e
            if tmp1 > tmp0:
                numer = tmp1 - tmp0
                denom = a - 2 * b + c
                if numer >= denom:
                    s = 1
                    t = 0
                    sqr_distance = a + 2 * d + f
                else:
                    s = numer / denom
                    t = 1 - s
                    sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f
            else:
                s = 0
                if tmp1 <= 0:
                    t = 1
                    sqr_distance = c + 2 * e + f
                else:
                    if e >= 0:
                        t = 0
                        sqr_distance = f
                    else:
                        t = -e / c
                        sqr_distance = e * t + f
            # end of region 2
        else:
            if t < 0:
                # region 6
                tmp0 = b + e
                tmp1 = a + d
                if tmp1 > tmp0:
                    numer = tmp1 - tmp0
                    denom = a - 2 * b + c
                    if numer >= denom:
                        t = 1
                        s = 0
                        sqr_distance = c + 2 * e + f
                    else:
                        t = numer / denom
                        s = 1 - t
                        sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f
                else:
                    t = 0
                    if tmp1 <= 0:
                        s = 1
                        sqr_distance = a + 2 * d + f
                    else:
                        if d >= 0:
                            s = 0
                            sqr_distance = f
                        else:
                            s = -d / a
                            sqr_distance = d * s + f
                # end of region 6
            else:
                # region 1
                numer = c + e - b - d
                if numer <= 0:
                    s = 0
                    t = 1
                    sqr_distance = c + 2 * e + f
                else:
                    denom = a - 2 * b + c
                    if numer >= denom:
                        s = 1
                        t = 0
                        sqr_distance = a + 2 * d + f
                    else:
                        s = numer / denom
                        t = 1 - s
                        sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f
                # end of region 1

    # account for numerical round-off error
    if sqr_distance < 0:
        sqr_distance = 0

    # return distance and closest point
    dist = np.sqrt(sqr_distance)
    pp0 = b_0 + s * e_0 + t * e_1

    return dist, pp0
