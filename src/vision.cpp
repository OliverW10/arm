#include "vision.hpp"

Vision::Vision()
{
    // rotation from t265 coordinate system to my coordinate system
    // https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    t265_to_camera << 0, 0, 1, 0,
                      1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 0, 1;
    // t265 returns 0 rotation as identity with y & z ones as -1

    got_first_frame = false;
    last_print = std::chrono::steady_clock::now();
    // TODO: use device changed callback to detect a device being plugged in
    auto devs = ctx.query_devices();
    if (devs.size() != 1)
    {
        std::cerr << "too many or no realsense cameras: " << devs.size() << "\n";
        has_device = false;
        return;
    }
    auto dev = devs.front();
    has_device = true;
    // get device id, needed to enable
    const char *serial_num_arr = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::string serial_num_str = std::string(serial_num_arr);
    std::cout << "serial number: " << serial_num_str << "\n";

    cfg.enable_device(serial_num_str);
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // have to enable both for some reason according to pose-and-image example
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    // use lambda to capture this
    pipe.start(cfg, [&](const rs2::frame& frame){realsenseLoop(frame);});
}

void Vision::realsenseLoop(const rs2::frame& frame)
{
    if(!got_first_frame){
        got_first_frame = true;
        std::cout << "got first realsense frame\n";
    }

    frame_pose_mutex.lock();
    frame_number = frame.get_frame_number();
    if(auto fp = frame.as<rs2::pose_frame>()){
        auto rs_camera_pose = fp.get_pose_data();
        camera_pose = poseToTransform(rs_camera_pose);
        std::cout << "raw camera pose:\n" << camera_pose << "\n";
    }else if(auto fs = frame.as<rs2::frameset>()){
    }

    frame_pose_mutex.unlock();
    last_frame = std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();
    if (now - last_print >= std::chrono::seconds(1))
    {
        last_print = now;
    }
}

void Vision::apriltagLoop()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool Vision::isActive()
{
    auto now = std::chrono::steady_clock::now();
    return got_first_frame && now - last_frame < std::chrono::milliseconds(1000);
}

Eigen::Matrix4d Vision::getArmPose()
{
    // releases mutex when it goes out of scope
    std::lock_guard<std::mutex> lock(frame_pose_mutex);
    return t265_to_camera * camera_pose;
}

Eigen::Matrix4d Vision::poseToTransform(const rs2_pose &rs_pose)
{
    // create matrix
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();

    // add pose rotation matrix to full matrix
    Eigen::Quaterniond quat(rs_pose.rotation.w, rs_pose.rotation.x, rs_pose.rotation.y, rs_pose.rotation.z);
    mat.block<3, 3>(0, 0) = quat.matrix();

    // mat(0, 3) = -rs_pose.translation.z;
    // mat(1, 3) = -rs_pose.translation.x;
    // mat(2, 3) = rs_pose.translation.y;
    mat(0, 3) = rs_pose.translation.x;
    mat(1, 3) = rs_pose.translation.y;
    mat(2, 3) = rs_pose.translation.z;
    return mat;
}

Eigen::Matrix4d Vision::cameraToArm(const Eigen::Matrix4d &camera)
{
    // std::cout << "camera:\n" << camera << "\n";
    return camera;
}
