#ifndef VISIONH
#define VISIONH

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <librealsense2/h/rs_types.h>
#include <kdl/frames.hpp>

class Vision{
public:
    Vision();
    void execute();
    KDL::Vector getPose();
private:
    // void device_callback();
    rs2::context ctx;
    rs2::pipeline pipe;
    rs2::config cfg;
    bool has_device;

    // the last video frame
    rs2::video_frame* last_frame;
    // the last pose 'frame'
    rs2::pose_frame* last_pose;
};

// coverts a rs vector to kdl vector
KDL::Vector RsToKdl(rs2::vec3d vector);

// converts a rs pose to kdl frame
KDL::Frame RsToKdl(rs2_pose pose);



#endif // VISION