#ifndef VISIONH
#define VISIONH

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <librealsense2/h/rs_types.h>
#include <Eigen/Dense>

class Vision{
public:
    Vision();
    void execute();
    rs2_pose getPose();
private:
    // void device_callback();
    rs2::context ctx;
    rs2::pipeline pipe;
    rs2::config cfg;
    bool has_device;

    rs2::frameset* last_frame;
    // // the last video frame
    // rs2::video_frame* last_frame;
    // // the last pose 'frame'
    // rs2::pose_frame* last_pose;
};

Eigen::Matrix4f poseToMat(rs2_pose rs_pose);

#endif // VISION