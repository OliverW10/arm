#include <Eigen/Core>
#include <iostream>

int main(){
    Eigen::Matrix4d test_mat = Eigen::Matrix4d::Identity();
    Eigen::Vector4d test_vec(0, 0.5, 5, 1);
    Eigen::Matrix4d output = test_mat * test_vec;
    std::cout << test_vec;
}