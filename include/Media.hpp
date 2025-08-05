#ifndef MEDIA_HPP
#define MEDIA_HPP

#include "CompilationFlags.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "Config.hpp"
#include "DepthMediaFile.hpp"
#include "Frame.hpp"

class Media
{
private:
    // Flags
    bool is_video;
    bool is_eof;

    // RGB media
    cv::Mat rgb_photo;
    cv::VideoCapture rgb_video_capture;
    uint32_t rgb_frame_width;
    uint32_t rgb_frame_height;

    // Ground truth depth media
    DepthMediaFile depth_media_file;

    // ML depth media
    MLDepthMediaFile ml_depth_media_file;

    // For supporting dynamic fps
    unsigned short rgb_frame_interval;
    uint32_t dynamic_fps_frame_interval = 0;  // in milliseconds, used for dynamic fps in depth video (read along with every depth frame)
    unsigned short accumulated_time_ms = 0; // in milliseconds and always below dynamic_fps_frame_interval, used for frame synchronization when dynamic fps is used in depth video
    

public:
    Media(const AnalyzerConfig &analyzer_config);
    ~Media();

    int get_next_frame(Frame &frame);
    unsigned short get_rgb_frame_interval();
};

#endif // MEDIA_HPP