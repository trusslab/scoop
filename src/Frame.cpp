#include "Frame.hpp"

Frame::Frame()
{
    is_cropped = false;
    cropping_width_offset_factor = 0.0;

    ml_depth_frame = &ml_depth_frame_original;
}

Frame::~Frame() { }

void Frame::preprocess_depth_frame()
{
    depth_frame.prepare_normalized_depth_frame();
    depth_frame.prepare_normalized_inverted_depth_frame();
}

void Frame::preprocess_ml_depth_frame()
{
    ml_depth_frame->prepare_normalized_depth_frame();
}

bool Frame::crop_ml_depth_frame_if_needed()
{
    if (int(float(depth_frame.depth_frame_raw.cols) / float(depth_frame.depth_frame_raw.rows)) != int(float(rgb_frame.cols) / float(rgb_frame.rows))) {
        double width_scaling_factor = (double(depth_frame.depth_frame_raw.cols) * (double(rgb_frame.rows) / double(depth_frame.depth_frame_raw.rows)) / double(rgb_frame.cols));
        cropping_width_offset_factor = (1.0 - width_scaling_factor) / 2.0;

#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE && defined ENABLE_DEBUG_LOG_VERBOSE_ML_DEPTH_FRAME_CROPPING
        std::cout << "Aspect ratio mismatched, cropping ML frame's width in center; width_offset_scaling_factor: " << cropping_width_offset_factor << std::endl;
#endif

        // Crop ML depth frame
        is_cropped = true;
        double ml_depth_frame_width_offset = cropping_width_offset_factor * ml_depth_frame_original.depth_frame_raw.cols;
        cv::Rect ml_depth_frame_crop_rect(ml_depth_frame_width_offset, 0, round(ml_depth_frame_original.depth_frame_raw.cols - 2 * ml_depth_frame_width_offset), ml_depth_frame_original.depth_frame_raw.rows);
        ml_depth_frame_cropped.depth_frame_raw = ml_depth_frame_original.depth_frame_raw(ml_depth_frame_crop_rect);
        ml_depth_frame_cropped.camera_intrinsics = ml_depth_frame_original.camera_intrinsics;
        ml_depth_frame = &ml_depth_frame_cropped;
    }

    return is_cropped;
}

void Frame::preprocess_point_clouds()
{
    depth_frame.prepare_point_cloud();
    ml_depth_frame->prepare_point_cloud();
    
#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE && defined ENABLE_DEBUG_LOG_VERBOSE_POINT_CLOUD_PREPROCESSING
    std::cout << "[Ground Truth]: There are a total of " << depth_frame.point_cloud->size() << " points in the depth frame with a resolution of ";
    std::cout << depth_frame.depth_frame_raw.cols << "x" << depth_frame.depth_frame_raw.rows << std::endl;
    std::cout << "[ML Truth]: There are a total of " << ml_depth_frame->point_cloud->size() << " points in the depth frame with a resolution of ";
    std::cout << ml_depth_frame->depth_frame_raw.cols << "x" << ml_depth_frame->depth_frame_raw.rows << std::endl;
#endif
}

void Frame::preprocess()
{
    preprocess_depth_frame();
    crop_ml_depth_frame_if_needed();
    preprocess_ml_depth_frame();
    preprocess_point_clouds();
}