#ifndef VISIONH
#define VISIONH

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <librealsense2/h/rs_types.h>
#include <Eigen/Dense>
#include <mutex>

class Vision{
public:
    Vision();
    void execute();
    rs2_pose getPose();
private:
    // void device_callback();
    void realsenseLoop();
    std::mutex frame_mutex;
    rs2::context ctx;
    rs2::pipeline pipe;
    rs2::config cfg;
    bool has_device;

    // the last video frame
    rs2::video_frame* last_video_frame;
    // the last pose 'frame'
    rs2_pose last_pose;
};

Eigen::Matrix4d poseToMat(const rs2_pose &rs_pose);

#endif // VISION