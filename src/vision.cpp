#include "vision.hpp"

Vision::Vision()
{
    // rotation from t265 coordinate system to my coordinate system
    // https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    t265_to_camera << 0, 0, 1, 0,
                      1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 0, 1;

    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE }, serial)){
        exit(EXIT_SUCCESS);
    }
    std::cout << "serial number: " << serial << "\n";

    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // was having issues with pose drifting off erratically large distances
    // this thread indicates its an issues with usb 2 and that not using the image streams helps
    // disabling the fisheye streams seemed to fix the issue
    // https://support.intelrealsense.com/hc/en-us/community/posts/360036423993/comments/360009213553
    // cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    // cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    // enable both streams according to pose-and-image example if you want one

    last_print = std::chrono::steady_clock::now();

    got_first_frame = false;
    // use lambda to capture this
    auto callback = [&](const rs2::frame& frame){realsenseLoop(frame);};
    pipe.start(cfg, callback);
}

// copied from realsense examples
bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
    rs2::context ctx;
    auto devs = ctx.query_devices();
    std::vector <rs2_stream> unavailable_streams = stream_requests;
    for (auto dev : devs)
    {
        std::map<rs2_stream, bool> found_streams;
        for (auto& type : stream_requests)
        {
            found_streams[type] = false;
            for (auto& sensor : dev.query_sensors())
            {
                for (auto& profile : sensor.get_stream_profiles())
                {
                    if (profile.stream_type() == type)
                    {
                        found_streams[type] = true;
                        unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
                        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                            out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    }
                }
            }
        }
        // Check if all streams are found in current device
        bool found_all_streams = true;
        for (auto& stream : found_streams)
        {
            if (!stream.second)
            {
                found_all_streams = false;
                break;
            }
        }
        if (found_all_streams)
            return true;
    }
    // After scanning all devices, not all requested streams were found
    for (auto& type : unavailable_streams)
    {
        switch (type)
        {
        case RS2_STREAM_POSE:
        case RS2_STREAM_FISHEYE:
            std::cerr << "Connect T26X and rerun the demo" << std::endl;
            break;
        case RS2_STREAM_DEPTH:
            std::cerr << "The demo requires Realsense camera with DEPTH sensor" << std::endl;
            break;
        case RS2_STREAM_COLOR:
            std::cerr << "The demo requires Realsense camera with RGB sensor" << std::endl;
            break;
        default:
            throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type
        }
    }
    return false;
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
    }else if(auto fs = frame.as<rs2::frameset>()){
        std::cout << "got fisheye frame\n";
    }else{
    }

    frame_pose_mutex.unlock();
    last_frame = std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();
    if (now - last_print >= std::chrono::milliseconds(1000))
    {
        last_print = now;
        // std::cout << "pose from callback:\n"
        //           << camera_pose.block<3, 1>(0, 3) << "\n\n";
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
