#ifndef VISIONH
#define VISIONH

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <librealsense2/h/rs_types.h>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
// #include "apriltag.h"

class Vision
{
public:
    Vision();
    Eigen::Matrix4d getPose();
    bool isActive();
private:
    // loops to run on other threads
    // waits for and reads new realsense frames
    void realsenseLoop();
    std::thread realsense_thread;

    // detects apriltags in the latest frame
    void apriltagLoop();
    std::thread apriltag_thread;

    // realsense objects
    rs2::context ctx;
    rs2::pipeline pipe;
    rs2::config cfg;
    bool has_device;

    // mutex for both the frame and pose
    std::mutex frame_mutex;
    // the last video frame
    rs2::video_frame *fisheye_frame;
    // the last pose 'frame'
    rs2_pose rs_camera_pose;
    unsigned long long frame_number;
    std::chrono::time_point<std::chrono::system_clock> last_frame;
    bool got_first_frame;

    Eigen::Matrix4d camera_pose;
};

Eigen::Matrix4d poseToTransform(const rs2_pose &rs_pose);

#endif // VISION