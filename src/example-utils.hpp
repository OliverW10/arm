// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 Intel Corporation. All Rights Reserved.

// #pragma once
#ifndef EXAMPLE_UTILS_H
#define EXAMPLE_UTILS_H

#include <iostream>
#include <string>
#include <map>
#include <librealsense2/rs.hpp>
#include <algorithm>

//////////////////////////////
// Demos Helpers            //
//////////////////////////////

// Find devices with specified streams
bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial);
#endif