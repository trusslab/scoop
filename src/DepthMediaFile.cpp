#include "DepthMediaFile.hpp"

// Depth media file class constructor
DepthMediaFile::DepthMediaFile()
{
    // Do nothing
}

// Depth media file class constructor
DepthMediaFile::DepthMediaFile(const std::string &filename)
{
    init(filename);
}

// Depth media file class constructor
DepthMediaFile::DepthMediaFile(const std::string &filename, const uint32_t &rgb_width, const uint32_t &rgb_height)
{
    init(filename, rgb_width, rgb_height);
}

// Depth media file class destructor
DepthMediaFile::~DepthMediaFile() {}

// Initialize the depth media file
void DepthMediaFile::init(const std::string &filename)
{
    depth_media_file_stream.open(filename, std::ios::binary);
    if (!depth_media_file_stream.is_open())
    {
        std::cerr << "Error: Could not open depth media file " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
    read_parameter(&camera_platform_id);
    switch (camera_platform_id) {
        case 0: // Android: Galaxy S20 Plus 5G (Snapdragon)
            camera_platform_endianness = 1;
            is_reading_depth_camera_intrinsics_dynamically = false;
            break;
        case 1: // iOS
            camera_platform_endianness = 0;
            is_reading_depth_camera_intrinsics_dynamically = true;
            break;
        default:
            std::cout << "Error: Unknown camera platform ID: " << camera_platform_id << std::endl;
            exit(EXIT_FAILURE);
    }
    read_parameter(&depth_width);
    read_parameter(&depth_height);
    read_parameter(&depth_pixel_size);
    read_parameter(&depth_fps);

    depth_frame_size = depth_width * depth_height * depth_pixel_size;

    // DEBUG: Print depth media file parameters
#ifdef ENABLE_DEBUG_LOG
    std::cout << "Depth media file parameters: ";
    std::cout << "camera_platform_id: " << (int)camera_platform_id;
    std::cout << "; depth_width: " << depth_width;
    std::cout << "; depth_height: " << depth_height;
    std::cout << "; depth_pixel_size: " << depth_pixel_size;
    std::cout << "; depth_fps: " << depth_fps << std::endl;
#endif // ENABLE_DEBUG_LOG
}

// Initialize the depth media file
void DepthMediaFile::init(const std::string &filename, const uint32_t &rgb_width, const uint32_t &rgb_height)
{
    init(filename);
    set_rgb_frame_size(rgb_width, rgb_height);
}

// Set the RGB frame size
void DepthMediaFile::set_rgb_frame_size(const uint32_t &rgb_width, const uint32_t &rgb_height)
{
    this->rgb_width = rgb_width;
    this->rgb_height = rgb_height;
    depth_width_ml = (uint32_t)round((float)depth_height * ((float)rgb_width / (float)rgb_height));
}

// Read a char parameter from the depth media file
void DepthMediaFile::read_parameter(char* parameter)
{
    depth_media_file_stream.read(parameter, sizeof(char));
}

// Read a unsigned 32-bit int parameter from the depth media file
void DepthMediaFile::read_parameter(uint32_t* parameter)
{
    depth_media_file_stream.read(reinterpret_cast<char *>(parameter), sizeof(uint32_t));
    if (camera_platform_endianness)
    {
        *parameter = bswap(*parameter);
    }
}

// Read a float parameter from the depth media file
void DepthMediaFile::read_parameter(float* parameter)
{
    depth_media_file_stream.read(reinterpret_cast<char *>(parameter), sizeof(float));
    if (camera_platform_endianness)
    {
        *parameter = bswap(*parameter);
    }
}

// Read a frame from the depth media file
void DepthMediaFile::read_frame(DepthFrame* depth_frame)
{
    // Read depth camera intrinsics
    if (is_reading_depth_camera_intrinsics_dynamically)
    {
        read_parameter(&depth_frame->camera_intrinsics.fx);
        read_parameter(&depth_frame->camera_intrinsics.fy);
        read_parameter(&depth_frame->camera_intrinsics.cx);
        read_parameter(&depth_frame->camera_intrinsics.cy);

        // Compute the depth camera intrinsics if the platform is iOS
        if (camera_platform_id == 1)
        {
            // Check if the RGB frame size is set
            if (rgb_width == 0 || rgb_height == 0)
            {
                std::cerr << "Error: RGB frame size is not set" << std::endl;
                exit(EXIT_FAILURE);
            }

            depth_frame->camera_intrinsics.fx = (float)depth_width / (float)rgb_width * depth_frame->camera_intrinsics.fx;
            depth_frame->camera_intrinsics.fy = (float)depth_height / (float)rgb_height * depth_frame->camera_intrinsics.fy;
            depth_frame->camera_intrinsics.cx = (float)depth_width / (float)rgb_width * depth_frame->camera_intrinsics.cx;
            depth_frame->camera_intrinsics.cy = (float)depth_height / (float)rgb_height * depth_frame->camera_intrinsics.cy;
        }
    }
    else
    {
        switch (camera_platform_id)
        {
            case 0: // Android: Galaxy S20 Plus 5G (Snapdragon)
                depth_frame->camera_intrinsics.fx = GALAXY_S20_TOF_CAMERA_FX_DEFAULT;
                depth_frame->camera_intrinsics.fy = GALAXY_S20_TOF_CAMERA_FY_DEFAULT;
                depth_frame->camera_intrinsics.cx = GALAXY_S20_TOF_CAMERA_CX_DEFAULT;
                depth_frame->camera_intrinsics.cy = GALAXY_S20_TOF_CAMERA_CY_DEFAULT;
                break;
            default:
                std::cout << "Error: Unknown camera platform ID: " << camera_platform_id << std::endl;
                exit(EXIT_FAILURE);
        }
    }
#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE && defined ENABLE_DEBUG_LOG_VERBOSE_DEPTH_FRAME_CAMERA_INTRINSICS
    std::cout << "Depth camera intrinsics(dynamic: " << is_reading_depth_camera_intrinsics_dynamically << "): ";
    std::cout << "fx: " << depth_frame->camera_intrinsics.fx;
    std::cout << "; fy: " << depth_frame->camera_intrinsics.fy;
    std::cout << "; cx: " << depth_frame->camera_intrinsics.cx;
    std::cout << "; cy: " << depth_frame->camera_intrinsics.cy << std::endl;
#endif

    // Allocate depth frame based on platform
    switch(camera_platform_id)
    {
        case 0: // Android: Galaxy S20 Plus 5G (Snapdragon)
            depth_frame->depth_frame_raw.create(depth_height, depth_width, CV_16UC1);
            break;
        case 1: // iOS
            depth_frame->depth_frame_raw.create(depth_height, depth_width, CV_16F);
            break;
        default:
            std::cout << "Error: Unknown camera platform ID: " << camera_platform_id << std::endl;
            exit(EXIT_FAILURE);
    }

    // Read depth frame
#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE && defined ENABLE_DEBUG_LOG_VERBOSE_DEPTH_FRAME_CAMERA_INTRINSICS
    std::cout << "Reading depth frame of size: " << depth_frame_size << std::endl;
#endif
    read(reinterpret_cast<char *>(depth_frame->depth_frame_raw.data), depth_frame_size);

    // Invalidate the first 3 bits of each pixel in depth frame if the platform is Android
    if (camera_platform_id == 0)
    {
        for (int i = 0; i < depth_frame->depth_frame_raw.rows; i++)
        {
            for (int j = 0; j < depth_frame->depth_frame_raw.cols; j++)
            {
                uint16_t pixel = depth_frame->depth_frame_raw.at<uint16_t>(i, j);
                pixel &= 0x1FFF;
                depth_frame->depth_frame_raw.at<uint16_t>(i, j) = pixel;
            }
        }
    }

    // Convert millimeters to meters if the platform is Android
    if (camera_platform_id == 0)
    {
        depth_frame->depth_frame_raw.convertTo(depth_frame->depth_frame_raw, CV_32F, 0.001f);
    }

    // Convert pixel type to 32-bit float
    depth_frame->depth_frame_raw.convertTo(depth_frame->depth_frame_raw, CV_32F);
}

// Arbitrary read from the depth media file
void DepthMediaFile::read(void* data, const uint32_t size)
{
    depth_media_file_stream.read(reinterpret_cast<char *>(data), size);
}

// Check if the end of the depth media file is reached
bool DepthMediaFile::is_eof()
{
    return depth_media_file_stream.eof();
}

// ML depth media file class constructor
MLDepthMediaFile::MLDepthMediaFile()
{
    // Do nothing
}

// ML depth media file class constructor
MLDepthMediaFile::MLDepthMediaFile(const std::string &filename)
{
    init(filename);
}

// ML depth media file class destructor
MLDepthMediaFile::~MLDepthMediaFile() {}

// Init the ML depth media file
void MLDepthMediaFile::init(const std::string &filename)
{
    depth_media_file_stream.open(filename, std::ios::binary);
    if (!depth_media_file_stream.is_open())
    {
        std::cerr << "Error: Could not open ML depth media file " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Read a frame from the ML depth media file
void MLDepthMediaFile::read_frame(DepthFrame* depth_frame, const uint32_t &frame_width, const uint32_t &frame_height, const int &cv_pixel_type)
{
    // Read depth frame
    depth_frame->depth_frame_raw.create(frame_height, frame_width, cv_pixel_type);
    uint32_t depth_frame_size = frame_width * frame_height * depth_frame->depth_frame_raw.elemSize();

    depth_media_file_stream.read(reinterpret_cast<char *>(depth_frame->depth_frame_raw.data), depth_frame_size);

    // Check if error occurred while reading the depth frame
    if (depth_media_file_stream.fail())
    {
        std::cerr << "Error: ML depth media file read failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Convert pixel type to 32-bit float if the pixel type is 16-bit float
    if (cv_pixel_type == CV_16F)
    {
        depth_frame->depth_frame_raw.convertTo(depth_frame->depth_frame_raw, CV_32F);
    }
}

// Read a frame from the ML depth media file (and copy camera intrinsics)
void MLDepthMediaFile::read_frame(DepthFrame* depth_frame, const uint32_t &frame_width, const uint32_t &frame_height, const CameraIntrinsics camera_intrinsics, const int &cv_pixel_type)
{
    // Copy camera intrinsics
    depth_frame->camera_intrinsics = camera_intrinsics;

    // Read depth frame
    read_frame(depth_frame, frame_width, frame_height, cv_pixel_type);
}

// Check if the end of the ML depth media file is reached
bool MLDepthMediaFile::is_eof()
{
    return depth_media_file_stream.eof();
}