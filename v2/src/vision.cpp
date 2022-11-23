#include "vision.hpp"

Vision::Vision()
{
    got_first_frame = false;
    // TODO: use device changed callback to detect a device being plugged in
    auto devs = ctx.query_devices();
    if (devs.size() == 0)
    {
        std::cerr << "No realsense camera found\n";
        has_device = false;
        return;
    }
    if (devs.size() > 1)
    {
        std::cerr << "Too many realsense cameras found\n";
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

    realsense_thread = std::thread(&Vision::realsenseLoop, this);
    apriltag_thread = std::thread(&Vision::apriltagLoop, this);

    //// Create transformation matricies between camera and arm
    // rotation from t265 coordinate system to my coordinate system
    // https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    Eigen::Matrix3d t265_to_camera_rotation;
    t265_to_camera_rotation = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d t265_to_camera = Eigen::Matrix4d::Identity();
    t265_to_camera.block<3,3>(0, 0) = t265_to_camera_rotation;

    // rotation from camera to arm
    Eigen::Matrix3d camera_to_arm_rotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix4d camera_to_arm = Eigen::Matrix4d::Identity();
    camera_to_arm.block<3, 3>(0, 0) = camera_to_arm_rotation;
    camera_to_arm.block<3, 1>(0, 3) = Eigen::Vector3d(-0.1, 0, 0.05);

    Eigen::Matrix4d t265_to_arm = t265_to_camera * camera_to_arm;
    std::cout << "t265 to arm:\n" << t265_to_arm;
}

void Vision::realsenseLoop()
{
    pipe.start(cfg);

    int fps_count = 0;
    auto last_print = std::chrono::steady_clock::now();

    rs2::frameset frames;
    while (true)
    {
        // blocks until frames are ready
        bool got_frame = pipe.poll_for_frames(&frames);
        if (got_frame)
        {
            got_first_frame = true;
            frame_pose_mutex.lock();

            // assign to variable so can be dereferanced
            auto temp_fisheye_frame = frames.get_fisheye_frame(1); // left camera
            fisheye_frame = &(temp_fisheye_frame);
            // fisheye_frame->keep();
            frame_number = fisheye_frame->get_frame_number();
            rs_camera_pose = frames.get_pose_frame().get_pose_data();

            // TODO: set by apriltags
            camera_pose = poseToTransform(rs_camera_pose);

            frame_pose_mutex.unlock();
            last_frame = std::chrono::steady_clock::now();

            fps_count += 1;
            auto now = std::chrono::steady_clock::now();
            if (now - last_print >= std::chrono::seconds(1))
            {
                // std::cout << "realsense fps: " << fps_count << "\n";
                // std::cout << "pose from rs thread:\n"
                //           << camera_pose << "\n\n";
                fps_count = 0;
                last_print = now;
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
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
    return got_first_frame && now - last_frame < std::chrono::milliseconds(500);
}

Eigen::Matrix4d Vision::getArmPose()
{
    // releases mutex when it goes out of scope
    std::lock_guard<std::mutex> lock(frame_pose_mutex);
    return camera_pose;
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


Eigen::Matrix4d Vision::cameraToArm(const rs2_pose &rs_pose){
    Eigen::Matrix4d camera = poseToTransform(rs_pose);
    // rotate to arm frame
    // TODO: check maths on all this
    return camera * t265_to_arm;
}