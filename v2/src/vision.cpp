#include "vision.hpp"

Vision::Vision()
{
    // TODO: use device changed callback to detect a device being plugged in
    auto devs = ctx.query_devices();
    if (devs.size() == 0)
    {
        std::cerr << "No realsense camera found\n";
        return;
    }
    if (devs.size() > 1)
    {
        std::cerr << "Too many realsense cameras found\n";
        return;
    }
    auto dev = devs.front();
    // complains if no const
    const char *serial_num_arr = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::string serial_num_str = std::string(serial_num_arr);
    std::cout << serial_num_str << "\n";
    cfg.enable_device(serial_num_str);
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // have to enable both for some reason according to pose-and-image example
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    pipe.start(cfg);
}

// should be called at >= 200hz
void Vision::execute()
{
    rs2::frameset *frame;

    bool got_frame = pipe.poll_for_frames(frame);
    if (got_frame)
    {
        last_frame = frame;
    }
}

rs2_pose Vision::getPose()
{
    return last_frame->get_pose_frame().get_pose_data();
}

Eigen::Matrix4f poseToMat(rs2_pose rs_pose){
    // create matrix

    // add pose rotation matrix to full matrix

    // add pose transformation to full matrix

    // set bottom row

}