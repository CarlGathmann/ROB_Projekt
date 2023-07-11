//
// Created by FredvomJupiter on 24.06.2023.
//

#include <iostream>
#include <vector>
#include <utility>
#include "stl_reader-master/stl_reader.h"
#include "header/force_distance.h"


std::vector<std::vector<std::vector<float>>> convert_stl(std::string filepath) {
    std::vector<float> coords;
    std::vector<std::vector<float>> triangle;
    std::vector<std::vector<std::vector<float>>> triangle_list;

    try {
        stl_reader::StlMesh <float, unsigned int> mesh (filepath);

        for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
            std::cout << "coordinates of triangle " << itri << ": ";
            triangle.clear();
            for(size_t icorner = 0; icorner < 3; ++icorner) {
                const float* c = mesh.tri_corner_coords (itri, icorner);
                // or alternatively:
                // float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
                coords.clear();
                coords.push_back(c[0]); coords.push_back(c[1]); coords.push_back(c[2]);
                triangle.push_back(coords);
                std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
            }
            triangle_list.push_back(triangle);
            std::cout << std::endl;

            const float* n = mesh.tri_normal(itri);
            std::cout   << "normal of triangle " << itri << ": "
                        << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
        }
        std::cout << "triangle amount: "<<triangle_list.size() <<  std::endl;
        std::cout << "triangle: "<<triangle.size() <<  std::endl;
        std::cout << "coords: "<< coords.size() <<  std::endl;
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    return triangle_list;
}


int main() {
    std::string filepath = "meshes/test.stl";
    std::vector<std::vector<std::vector<float>>> triangle_list;
    triangle_list = convert_stl(filepath);
    //vectors should be eigen::vector3d for easy calculations
    double force_scalar = 0.0;
    std::vector<double> vec = {0.0, 0.0, 0.0};
    std::vector<double> point = { 50.0, 50.0, 0.0 };    // position from robot TCP
    std::pair<double, std::vector<double>> dist_foot;
    for (auto & i : triangle_list) {
        std::vector<std::vector<float>> triangle = i;
        dist_foot = point_triangle_distance(i, point);

        force_scalar += force(std::get<0>(dist_foot), 0.02, 0.05);
        //vec += std::get<1>>(dist_foot) - point;
    }
    std::cout <<"force: " << force_scalar << std::endl;
}

