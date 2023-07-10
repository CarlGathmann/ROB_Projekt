//
// Created by FredvomJupiter on 24.06.2023.
//

#include <iostream>
#include <vector>
#include "stl_reader-master/stl_reader.h"
//#include "matplotlib-cpp-master/matplotlibcpp.h"

std::vector<std::vector<float>> convert_stl(std::string filepath) {
    std::vector<float> coord;
    std::vector<std::vector<float>> triangle_list;
    try {
        stl_reader::StlMesh <float, unsigned int> mesh (filepath);

        for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
            std::cout << "coordinates of triangle " << itri << ": ";
            for(size_t icorner = 0; icorner < 3; ++icorner) {
                const float* c = mesh.tri_corner_coords (itri, icorner);
                // or alternatively:
                // float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
                coord.clear();
                coord.push_back(c[0]); coord.push_back(c[1]); coord.push_back(c[2]);
                std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
            }
            triangle_list.push_back(coord);
            std::cout << std::endl;

            const float* n = mesh.tri_normal(itri);
            std::cout   << "normal of triangle " << itri << ": "
                        << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
        }
        std::cout << "triangle amount: "<<triangle_list.size() <<  std::endl;
        std::cout << "coord: "<< coord.size() <<  std::endl;
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    return triangle_list;
}


 //TODO: create distance to triangle function
 //TODO: create force function

int main(){
    std::string filepath = "meshes/test.stl";
    std::vector<std::vector<float>> triangle_list;
    triangle_list = convert_stl(filepath);
}

