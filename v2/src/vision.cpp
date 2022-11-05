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
    // get device id, needed to enable
    const char *serial_num_arr = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::string serial_num_str = std::string(serial_num_arr);
    std::cout << serial_num_str << "\n";

    cfg.enable_device(serial_num_str);
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // have to enable both for some reason according to pose-and-image example
    // cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    // cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    pipe.start(cfg);
}

void Vision::execute()
{
}

void Vision::realsenseLoop(){
    while(true){
        // automatically frees mutex when it goes out of scope
        std::lock_guard<std::mutex> lock(frame_mutex);

        auto frames        = pipe.wait_for_frames();
        auto fisheye_frame = frames.get_fisheye_frame(fisheye_sensor_idx);
        auto frame_number  = fisheye_frame.get_frame_number();
        auto camera_pose   = frames.get_pose_frame().get_pose_data();
        // trys to get a pose frame
        bool got_pose = pipe.try_wait_for_frames(&frames);
        // check if frame is a pose frame
        frames
        if(rs2::pose_frame pf = frames.as<rs2::pose_frame>()){

        }
    }
}

rs2_pose Vision::getPose()
{
    return last_pose;
}

Eigen::Matrix4d poseToMat(const rs2_pose &rs_pose){
    // create matrix
    Eigen::Matrix4d mat;

    // add pose rotation matrix to full matrix
    Eigen::Quaterniond quat(rs_pose.rotation.w, rs_pose.rotation.x, rs_pose.rotation.y, rs_pose.rotation.z);
    mat.block<3, 3>(0, 0) = quat.matrix();

    // add pose transformation to full matrix
    mat(0, 3) = rs_pose.translation.x;
    mat(1, 3) = rs_pose.translation.x;
    mat(2, 3) = rs_pose.translation.x;

    // set bottom row
    mat(3, 0) = 0;
    mat(3, 1) = 0;
    mat(3, 2) = 0;
    mat(3, 3) = 1;
    return mat;
}