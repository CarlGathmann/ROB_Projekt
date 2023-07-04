//
// Created by FredvomJupiter on 24.06.2023.
//
#include <iostream>
#include "stl_reader-master/stl_reader.h"
//#include "matplotlib-cpp-master/matplotlibcpp.h"

int main(){
    wchar_t* filename = reinterpret_cast<wchar_t *>('meshes/Test ROB Proj v2.stl');

        stl_reader::StlMesh <float, unsigned int> mesh(filename);

        for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
            std::cout << "coordinates of triangle " << itri << ": ";
            for(size_t icorner = 0; icorner < 3; ++icorner) {
                const float* c = mesh.tri_corner_coords (itri, icorner);
                // or alternatively:
                // float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));
                std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
            }
            std::cout << std::endl;

            float* n = const_cast<float *>(mesh.tri_normal(itri));
            std::cout   << "normal of triangle " << itri << ": "
                        << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
        }
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}