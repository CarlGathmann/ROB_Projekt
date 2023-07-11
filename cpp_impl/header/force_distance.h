//
// Created by FredvomJupiter on 11.07.2023.
//

#ifndef ROB_PROJEKT_FORCE_DISTANCE_H
#define ROB_PROJEKT_FORCE_DISTANCE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>

double dot(std::vector<double> v1, std::vector<double> v2) {
    return std::inner_product(v1.begin(), v1.end(), v2.begin(), 0);
}

std::pair<double, std::vector<double>> point_triangle_distance(const std::vector<std::vector<float>>& tri, const std::vector<double>& p) {
    // vectors
    std::vector<float> B = tri[0];
    std::vector<double> E0 = {tri[1][0] - B[0], tri[1][1] - B[1], tri[1][2] - B[2]};
    std::vector<double> E1 = {tri[2][0] - B[0], tri[2][1] - B[1], tri[2][2] - B[2]};
    std::vector<double> D = {B[0] - p[0], B[1] - p[1], B[2] - p[2]};

    // dot products
    double a = dot(E0, E0);
    double b = dot(E0, E1);
    double c = dot(E1, E1);
    double d = dot(E0, D);
    double e = dot(E1, D);
    double f = dot(D, D);

    double det = a * c - b * b;
    double s = b * e - c * d;
    double t = b * d - a * e;
    double sqr_distance;

    if (s + t <= det) {
        if (s < 0) {
            if (t < 0) {
                // region 4
                if (d < 0) {
                    t = 0;
                    if (-d >= a) {
                        s = 1; sqr_distance = a + 2 * d + f;
                    } else {
                        s = -d / a; sqr_distance = d * s + f;
                    }
                } else {
                    s = 0;
                    if (e >= 0) {
                        t = 0; sqr_distance = f;
                    } else {
                        if (-e >= c) {
                            t = 1; sqr_distance = c + 2 * e + f;
                        } else {
                            t = -e / c; sqr_distance = e * t + f;
                        }
                    }
                }
                // end of region 4
            } else {
                // region 3
                s = 0;
                if (e >= 0) {
                    t = 0; sqr_distance = f;
                } else {
                    if (-e >= c) {
                        t = 1; sqr_distance = c + 2 * e + f;
                    } else {
                        t = -e / c; sqr_distance = e * t + f;
                    }
                }
                // end of region 3
            }
        } else {
            if (t < 0) {
                // region 5
                t = 0;
                if (d >= 0) {
                    s = 0; sqr_distance = f;
                } else {
                    if (-d >= a) {
                        s = 1; sqr_distance = a + 2 * d + f;
                    } else {
                        s = -d / a; sqr_distance = d * s + f;
                    }
                }
                // end of region 5
            } else {
                // region 0
                double inv_det = 1.0 / det;
                s = s * inv_det;
                t = t * inv_det; sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f;
            }
        }
    } else {
        if (s < 0) {
            // region 2
            double tmp0 = b + d;
            double tmp1 = c + e;
            if (tmp1 > tmp0) {
                double numer = tmp1 - tmp0;
                double denom = a - 2 * b + c;
                if (numer >= denom) {
                    s = 1;
                    t = 0; sqr_distance = a + 2 * d + f;
                } else {
                    s = numer / denom;
                    t = 1 - s; sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f;
                }
            } else {
                s = 0;
                if (tmp1 <= 0) {
                    t = 1; sqr_distance = c + 2 * e + f;
                } else {
                    if (e >= 0) {
                        t = 0; sqr_distance = f;
                    } else {
                        t = -e / c; sqr_distance = e * t + f;
                    }
                }
            }
            // end of region 2
        } else {
            if (t < 0) {
                // region 6
                double tmp0 = b + e;
                double tmp1 = a + d;
                if (tmp1 > tmp0) {
                    double numer = tmp1 - tmp0;
                    double denom = a - 2 * b + c;
                    if (numer >= denom) {
                        t = 1;
                        s = 0; sqr_distance = c + 2 * e + f;
                    } else {
                        t = numer / denom;
                        s = 1 - t; sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f;
                    }
                } else {
                    t = 0;
                    if (tmp1 <= 0) {
                        s = 1; sqr_distance = a + 2 * d + f;
                    } else {
                        if (d >= 0) {
                            s = 0; sqr_distance = f;
                        } else {
                            s = -d / a; sqr_distance = d * s + f;
                        }
                    }
                }
                // end of region 6
            } else {
                // region 1
                double numer = c + e - b - d;
                if (numer <= 0) {
                    s = 0;
                    t = 1; sqr_distance = c + 2 * e + f;
                } else {
                    double denom = a - 2 * b + c;
                    if (numer >= denom) {
                        s = 1;
                        t = 0; sqr_distance = a + 2 * d + f;
                    } else {
                        s = numer / denom;
                        t = 1 - s; sqr_distance = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f;
                    }
                }
                // end of region 1
            }
        }

    }

    // account for numerical round-off error
    if (sqr_distance < 0) {
        sqr_distance = 0;
    }
    // return distance and closest point
    double dist = std::sqrt(sqr_distance);
    std::vector<double> PP0 = {B[0] + s * E0[0] + t * E1[0], B[1] + s * E0[1] + t * E1[1], B[2] + s * E0[2] + t * E1[2]};

    return std::make_pair(dist, PP0);
}

/**
 * Creates a linear continuous function to calculate the feedback force with respect to the distance.
 * It is possible to modify the softness and sharpness of the object with full_force and no_force values.
 * @param dist distance robot TCP to virtual 3D object
 * @param full_force max force at desired distance. 0 < full_force.
 * @param no_force no force at desired distance full_force < no_force.
 * @return force value to be used as scalar.
 */
double force(double dist, double full_force, double no_force) {
    double interval = no_force - full_force;

    if (0.0 <= dist <= full_force) {
        return 1.0;
    }else if (full_force < dist <= no_force) {
        return -(1 / interval) * dist + (no_force / interval);
    }else if(dist > no_force){
        return 0;
    }
}

#endif //ROB_PROJEKT_FORCE_DISTANCE_H
