//
// Created by FredvomJupiter on 11.07.2023.
//

#ifndef ROB_PROJEKT_FORCE_DISTANCE_H
#define ROB_PROJEKT_FORCE_DISTANCE_H

#include <iostream>
#include <Eigen/Dense>
#include "stl_reader-master/stl_reader.h"
#include <cmath>
#include <vector>
#include <numeric>


/**
 * converts a stl file to a list of triangles using the stl_reader.h
 * the triangles are represented thru their vertices.
 * https://github.com/sreiter/stl_reader
 * @param filepath string with the stl-file path
 * @return vector filled with all triangles of the stl-file
 */
std::vector<Eigen::Matrix3d> convert_stl(std::string filepath) {
    Eigen::Matrix3d triangle;
    std::vector<Eigen::Matrix3d> triangle_list;

    try {
        stl_reader::StlMesh <float, unsigned int> mesh (filepath);

        for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
            //std::cout << "coordinates of triangle " << itri << ": ";

            for(int icorner = 0; icorner < 3; ++icorner) {
                const float* c = mesh.tri_corner_coords (itri, icorner);
                // or alternatively:
                // float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner))
                //std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
                triangle.col(icorner) << c[0], c[1], c[2];
            }
            triangle_list.push_back(triangle);
            /**
            std::cout << std::endl;

            const float* n = mesh.tri_normal(itri);
            std::cout   << "normal of triangle " << itri << ": "
                        << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
             **/
        }
        //std::cout << "triangle amount: "<<triangle_list.size() <<  std::endl;
        //std::cout << "triangle: "<<triangle.size() <<  std::endl;
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    return triangle_list;
}

int point_triangle_distance(Eigen::Matrix3d& tri, Eigen::Vector3d& p, Eigen::Vector3d* vec_out_ptr, double* dist_ptr) {
    // vectors
    Eigen::Vector3d B = tri.col(0);
    Eigen::Vector3d E0 = {tri.col(1)[0] - B[0], tri.col(1)[1] - B[1], tri.col(1)[2] - B[2]};
    Eigen::Vector3d E1 = {tri.col(2)[0] - B[0], tri.col(2)[1] - B[1], tri.col(2)[2] - B[2]};
    Eigen::Vector3d D = {B[0] - p[0], B[1] - p[1], B[2] - p[2]};

    // dot products

    double a = E0.dot(E0);
    double b = E0.dot(E1);
    double c = E1.dot(E1);
    double d = E0.dot(D);
    double e = E1.dot(D);
    double f = D.dot(D);

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
    *dist_ptr= std::sqrt(sqr_distance);
    *vec_out_ptr = {B[0] + s * E0[0] + t * E1[0], B[1] + s * E0[1] + t * E1[1], B[2] + s * E0[2] + t * E1[2]};
    return 0;
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
        return 0.0;
    }
    return 0.0;
}
/**
 * computes the force feedback as a vector in order to the given triangles(3D-Object)
 * @param triangle_list triangles of the stl_file
 * @param tcp_pos TCP posistion of the robot
 * @return feedback vector
 */
void feedback_vector(std::vector<Eigen::Matrix3d> triangle_list, Eigen::Vector3d tcp_pos, Eigen::Vector3d* feedback_vec_ptr) {
    Eigen::Vector3d vec;
    double distance;
    double force_scalar;
    for (auto & triangle : triangle_list) {
       point_triangle_distance(triangle, tcp_pos, &vec, &distance);
        *feedback_vec_ptr += (vec - tcp_pos) * force(distance, 0.02, 0.05);;
    }

}
#endif //ROB_PROJEKT_FORCE_DISTANCE_H
