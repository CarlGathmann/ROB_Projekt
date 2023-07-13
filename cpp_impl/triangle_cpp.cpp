//
// Created by FredvomJupiter on 24.06.2023.
//

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include "header/force_distance.h"


int main() {
    std::string filepath = "meshes/cube.stl";
    std::vector<Eigen::Matrix3d> triangle_list;
    triangle_list = convert_stl(filepath);
    double force_scalar = 0.0;
    Eigen::Vector3d vec;
    Eigen::Vector3d point = {0.0, 0.67, 0.67 };    // position from robot TCP
    std::pair<double, Eigen::Vector3d> dist_foot;
    for (int i = 0; i < triangle_list.size(); i++) {
        std::cout << "Coords of triangle " << i << ": \n";
        std::cout << triangle_list[i] << std::endl;
    }
    vec = feedback_vector(triangle_list, point);
    /**
    for (auto & triangle : triangle_list) {
        dist_foot = point_triangle_distance(triangle, point);
        force_scalar += force(std::get<0>(dist_foot), 0.02, 0.05);
        std::cout <<"distance: " << std::get<0>(dist_foot) << std::endl;
        vec += std::get<1>(dist_foot) - point;
    }
    std::cout <<"vector object to TCP: \n" << vec << std::endl;
    std::cout <<"vector object to TCP normalized: \n" << vec.normalized() << std::endl;
    std::cout <<"distance object to TCP: \n" << vec.norm() << std::endl;
    std::cout <<"force: " << force_scalar << std::endl;
    vec *= force_scalar;
    **/
    return 0;
}

