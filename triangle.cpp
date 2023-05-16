#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
/**
 * creates a rotation matrix and with it a coordinate system with one point of the triangle in the origin
 * and the other points in the xy plane.
 * @param triangle
 * @param rot_matrix_ptr
 */
void create_transformation_matrix(Vector3d triangle[], Matrix4d* transform_matrix_ptr){
    // declaration of all needed variables
    Vector3d x_axis;
    Vector3d y_axis;
    Vector3d z_axis;
    Vector3d helper;
    Matrix3d rot_matrix;
    Vector3d translation;
    Matrix4d transformation_matrix;
    // define one pair of points as one axis
    x_axis << (-triangle[0] + triangle[1]).normalized();
    std::cout << "x: " << x_axis.transpose() <<std::endl;
    // another pair of points to calculate their normal
    helper << (-triangle[0] + triangle[2]).normalized();
    // the normal of x_axis and another pair of points defines the z_axis
    z_axis << x_axis.cross(helper).normalized();
    std::cout << "z: " << z_axis.transpose() <<std::endl;
    // y_axis is defined by the norm of the already existing axis
    y_axis << x_axis.cross(z_axis).normalized();
    std::cout << "y:" << y_axis.transpose() <<std::endl;
    rot_matrix << x_axis, y_axis, z_axis;
    rot_matrix.transpose();

    // make sure the system is right-handed
    std::cout<< "det: " << rot_matrix.determinant()<< std::endl;
    if(rot_matrix.determinant() == -1) {
        std::cout << "Matrix is now right handed." << std::endl;
        y_axis *= -1;
        rot_matrix << x_axis, y_axis, z_axis;
    }

    transformation_matrix.topLeftCorner(rot_matrix.rows(), rot_matrix.cols()) = rot_matrix;
    transformation_matrix.col(3) << -triangle[0], 1;
    transformation_matrix.row(3) << 0, 0, 0, 1;
    // write the rot_matrix_ptr value with the calculated rotation matrix
    *transform_matrix_ptr = transformation_matrix;
}
/**
 * transformation of a point in the world or robots coordinate system into the coordinate system of the triangle
 * @param transformation_matrix
 * @param point
 * @param transformed_point_ptr
 */
void transform_point(Matrix4d transformation_matrix, Vector3d point, Vector3d* transformed_point_ptr){
    Vector4d point_4d;
    point_4d << point, 1;
    point_4d = point_4d.transpose() * transformation_matrix.inverse();
    *transformed_point_ptr = point_4d.head(point_4d.size() -1);
}
/**
 * the location of the point in order to one line of the triangle determined with the value of the variable location
 * it is mandatory to run clockwise around the triangle
 * if the value is
 * > 0: right hand side
 * = 0: on the line
 * < 0: left hand side
**/
void edge_function(Vector3d triangle[], Vector3d point_rel_tri, Array3d* locations){
    Vector3d a;
    a << triangle[2] - triangle[0];
    Vector3d b;
    b << triangle[1] - triangle[2];
    Vector3d c;
    c << triangle[0] - triangle[1];
    // along the edge a
    *locations << (point_rel_tri[0] - triangle[0][0]) * a[1] - (point_rel_tri[1] - triangle[0][1]) * a[0],
    // along the edge b
    (point_rel_tri[0] - triangle[2][0]) * b[1] - (point_rel_tri[1] - triangle[2][1]) * b[0],
    // along the edge c
    (point_rel_tri[0] - triangle[1][0]) * c[1] - (point_rel_tri[1] - triangle[1][1]) * c[0];
    std::cout << "vec a: " << a << std::endl;
    std::cout << "tri 2: " << triangle[2] << std::endl;
}

void determine_location(Array3d locations){
    for (int i = 0; i < locations.size(); i++) {
        if (locations[i] > 0){

        }
    }
}

double generate_vectors(Vector3d vector, char x_ory_y) {
    vector = vector * vector.norm();
}

int main() {
    Vector3d triangle [3];
    triangle[0] << 0, 0, 0;
    triangle[1] << 2, 0, 0;
    triangle[2] << 0, 2, 0;
    Matrix4d transform_matrix;
    Vector3d point;
    point << 1, 1, 0;
    Vector3d transformed_point;
    Array3d locations;

    create_transformation_matrix(triangle, &transform_matrix);
    transform_point(transform_matrix, point, &transformed_point);
    std::cout << transform_matrix <<std::endl;
    std::cout <<"point in relation to triangle: " << transformed_point.transpose() << std::endl;
    edge_function(triangle, transformed_point, &locations);
    std::cout <<"locations: " << locations.transpose() << std::endl;

    return 0;
}