#include "Media.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>  // Resize

Media::Media(const AnalyzerConfig &analyzer_config)
{
    // Check if the input is a video or a photo
    this->is_video = analyzer_config.is_processing_rgb_video;

    // Load the RGB photo or video
    if (is_video) {
        this->rgb_video_capture.open(analyzer_config.rgb_media_file_name);
        if (!rgb_video_capture.isOpened()) {
            std::cout << "Error: Could not open RGB video file." << std::endl;
            exit(-1);
        }
        this->rgb_frame_width = this->rgb_video_capture.get(cv::CAP_PROP_FRAME_WIDTH);
        this->rgb_frame_height = this->rgb_video_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    } else {
        this->rgb_photo = cv::imread(analyzer_config.rgb_media_file_name);
        if (rgb_photo.empty()) {
            std::cout << "Error: Could not open RGB photo file." << std::endl;
            exit(-1);
        }
        this->rgb_frame_width = rgb_photo.cols;
        this->rgb_frame_height = rgb_photo.rows;
    }

    // Load the ground truth depth media
    this->depth_media_file.init(analyzer_config.depth_media_file_name, this->rgb_frame_width, this->rgb_frame_height);
#ifdef ENABLE_DEBUG_LOG
    std::cout << "camera_platform_id: " << (int)(this->depth_media_file.camera_platform_id) << std::endl;
    std::cout << "depth resolution: " << this->depth_media_file.depth_width << "x" << this->depth_media_file.depth_height << std::endl;
    std::cout << "depth pixel size: " << this->depth_media_file.depth_pixel_size << "; depth frame size: " << this->depth_media_file.depth_frame_size << std::endl;
    std::cout << "depth video's fps: " << this->depth_media_file.depth_fps << std::endl;
#endif

    // Check if depth video's fps matches RGB video's fps (if not dynamic)
    if (this->is_video)
    {
        double rgb_fps = this->rgb_video_capture.get(cv::CAP_PROP_FPS);
        rgb_frame_interval = std::round(1000 / rgb_fps); // in milliseconds
    #ifdef ENABLE_DEBUG_LOG
        std::cout << "RGB video's fps: " << rgb_fps << "; frame interval: " << rgb_frame_interval << "ms" << std::endl;
    #endif
        if (this->depth_media_file.depth_fps != 0 && this->depth_media_file.depth_fps != std::round(rgb_fps)) {
            std::cout << "Error: Depth video's fps ("; 
            std::cout << this->depth_media_file.depth_fps;
            std::cout << ") does not match RGB video's fps (";
            std::cout << rgb_fps;
            std::cout << ")." << std::endl;
            exit(-1);
        }
    }

    // Load the ML depth media
    this->ml_depth_media_file.init(analyzer_config.ml_depth_media_file_name);

    // Other initializations
    this->is_eof = false;
}

Media::~Media()
{
    if (is_video) {
        this->rgb_video_capture.release();
    }
}

int Media::get_next_frame(Frame &frame)
{
    // Return 0 if frame is successfully read, 1 if end of media file is reached

    if (this->is_eof) {
        std::cout << "End of media file." << std::endl;
        return 1;
    }

    // Read the rgb frame
    if (this->is_video) {
        this->rgb_video_capture >> frame.rgb_frame;
        if (frame.rgb_frame.empty()) {
            std::cout << "Reached end of RGB video file." << std::endl;
            return 1;
        }
    } else {
        frame.rgb_frame = this->rgb_photo.clone();
        this->is_eof = true;
    }

    // Read the depth frame
    if (this->depth_media_file.is_eof()) {
        std::cout << "Reached end of depth media file." << std::endl;
        return 1;
    }
    if (this->is_video) {
        if (this->depth_media_file.depth_fps == 0) {
            while (accumulated_time_ms >= dynamic_fps_frame_interval) {
                accumulated_time_ms -= dynamic_fps_frame_interval;
                this->depth_media_file.read_parameter(&dynamic_fps_frame_interval);
                this->depth_media_file.read_frame(&(frame.depth_frame));
            }
            accumulated_time_ms += rgb_frame_interval;
        } else {
            this->depth_media_file.read_frame(&(frame.depth_frame));
        }
    } else {
        this->depth_media_file.read_frame(&(frame.depth_frame));
    }

    // Read ML depth frame
    if (this->ml_depth_media_file.is_eof()) {
        std::cout << "Reached end of ML depth media file." << std::endl;
        return 1;
    }
    // std::cout << "Expected ml depth frame size: " << this->depth_media_file.depth_width_ml << "x" << this->depth_media_file.depth_height << std::endl;
    this->ml_depth_media_file.read_frame(frame.ml_depth_frame, this->depth_media_file.depth_width_ml, this->depth_media_file.depth_height, frame.depth_frame.camera_intrinsics);

#if defined ENABLE_LOW_RES_DEPTH_FRAME_FOR_IOS_PLATFORM
    // FOR EXPERIMENTS: resize the ios depth frame to new width: 320
    if (this->depth_media_file.camera_platform_id == 1)
    {
        // Update parameters
        this->depth_media_file.depth_width = 320;
        this->depth_media_file.depth_width_ml = 320;
        this->depth_media_file.depth_height = 180;
        this->depth_media_file.depth_frame_size = this->depth_media_file.depth_width * this->depth_media_file.depth_height * frame.depth_frame.depth_frame_raw.elemSize();

        // Resize the depth frame
        std::cout << "DEBUG: Resizing the depth frame to new width: 320" << std::endl;
        cv::resize(frame.depth_frame.depth_frame_raw, frame.depth_frame.depth_frame_raw, cv::Size(this->depth_media_file.depth_width, this->depth_media_file.depth_height), 0, 0, cv::INTER_NEAREST);

        // Resize the ml depth frame
        cv::resize(frame.ml_depth_frame->depth_frame_raw, frame.ml_depth_frame->depth_frame_raw, cv::Size(this->depth_media_file.depth_width, this->depth_media_file.depth_height), 0, 0, cv::INTER_NEAREST);
    }
#endif // ENABLE_LOW_RES_DEPTH_FRAME_FOR_IOS_PLATFORM

    // Normalize depth frame
    frame.depth_frame.prepare_normalized_depth_frame();

    // Invert normalized depth frame (for matching with ML depth frame)
    frame.depth_frame.prepare_normalized_inverted_depth_frame();

    // Align the depth frame with the ML depth frame
    frame.crop_ml_depth_frame_if_needed();

    // Normalize ML depth frame
    frame.ml_depth_frame->prepare_normalized_depth_frame();

    // DEBUG: Print average depth
#if defined ENABLE_DEBUG_LOG_VERBOSE_AVG_DEPTH
    std::cout << "[Ground Truth]: ";
    frame.depth_frame.print_average_depth();
    std::cout << "[ML Truth]: ";
    frame.ml_depth_frame->print_average_depth();
#endif

    // Prepare point clouds
    frame.preprocess_point_clouds();
#if defined ENABLE_DEBUG_LOG_VERBOSE
    std::cout << "There are a total of " << frame.ml_depth_frame->point_cloud->size() << " points in the ml depth frame with a resolution of ";
    std::cout << frame.ml_depth_frame->depth_frame_raw.cols << "x" << frame.ml_depth_frame->depth_frame_raw.rows << std::endl;
#endif // ENABLE_DEBUG_LOG_VERBOSE

    return 0;
}

unsigned short Media::get_rgb_frame_interval()
{
    return this->rgb_frame_interval;
}