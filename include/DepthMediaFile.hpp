#ifndef DEPTH_VIDEO_FILE_HPP
#define DEPTH_VIDEO_FILE_HPP

#include <iostream>
#include <fstream>
#include <cstdint>
#include <string>

#include "CompilationFlags.hpp"
#include "DepthFrame.hpp"
#include "Utils.hpp"
#include "CameraParams.hpp"

// Depth media file class
class DepthMediaFile
{
    public:
    std::ifstream depth_media_file_stream;
    char camera_platform_id = -1; // platform ID that was used to capture the video; 0 means Android (Galaxy S20 Plus 5G), 1 means iOS
    bool camera_platform_endianness; // endianness of the platform that was used to capture the video; 0 means little, 1 means big
    uint32_t rgb_width;   // Width of RGB frame (needed to be set manually)
    uint32_t rgb_height;  // Height of RGB frame (needed to be set manually)
    uint32_t depth_width;   // Width of depth frame
    uint32_t depth_width_ml;  // Width of ml depth frame (to align with RGB frame) (calculated automatically)
    uint32_t depth_height;  // Height of depth frame
    uint32_t depth_pixel_size;  // Size of each pixel in bytes
    uint32_t depth_frame_size;  // Size of depth frame in bytes
    uint32_t depth_fps;  // 0 means dynamic frame rate
    bool is_reading_depth_camera_intrinsics_dynamically; // true means reading depth camera intrinsics dynamically

    DepthMediaFile();
    DepthMediaFile(const std::string &filename);
    DepthMediaFile(const std::string &filename, const uint32_t &rgb_width, const uint32_t &rgb_height);

    void init(const std::string &filename);
    void init(const std::string &filename, const uint32_t &rgb_width, const uint32_t &rgb_height);

    void set_rgb_frame_size(const uint32_t &rgb_width, const uint32_t &rgb_height);

    void read_parameter(char* parameter);

    void read_parameter(uint32_t* parameter);

    void read_parameter(float* parameter);

    void read_frame(DepthFrame* depth_frame);

    void read(void* data, const uint32_t size);

    bool is_eof();

    ~DepthMediaFile();
};

// ML depth media file class
class MLDepthMediaFile
{
    public:
    std::ifstream depth_media_file_stream;

    MLDepthMediaFile();
    MLDepthMediaFile(const std::string &filename);

    void init(const std::string &filename);

    void read_frame(DepthFrame* depth_frame, const uint32_t &frame_width, const uint32_t &frame_height, const int &cv_pixel_type=CV_16F);
    
    void read_frame(DepthFrame* depth_frame, const uint32_t &frame_width, const uint32_t &frame_height, const CameraIntrinsics camera_intrinsics, const int &cv_pixel_type=CV_16F);

    bool is_eof();

    ~MLDepthMediaFile();
};

#endif // DEPTH_VIDEO_FILE_HPP