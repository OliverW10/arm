#include <Eigen/Core>
#include <iostream>

int main(){
    Eigen::Matrix4d t265_to_camera;
    t265_to_camera << 0, 0, -1, 0,
                      1, 0, 0, 0,
                      0, -1, 0, 0,
                      0, 0, 0, 1;    
    
    Eigen::Matrix4d pose;
    pose.setIdentity();
    pose(0, 3) = 0.5;
    pose(1, 3) = 1;
    pose(2, 3) = 2;
    std::cout << "pose orig:\n" << pose << "\n";

    Eigen::Matrix4d res = t265_to_camera * pose;
    std::cout << "res:\n" << res << "\n";
}