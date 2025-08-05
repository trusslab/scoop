#ifndef FRAME_HPP
#define FRAME_HPP

#include "CompilationFlags.hpp"

#include <opencv2/core.hpp>

#include "DepthFrame.hpp"

// Frame class
class Frame
{
    public:
    // basic
    cv::Mat rgb_frame;
    DepthFrame depth_frame;
    DepthFrame* ml_depth_frame;
    DepthFrame ml_depth_frame_original;

    // cropped: if the depth frame cannot cover the whole rgb frame, 
    // we crop the rgb and ml frame to match the depth frame's aspect ratio 
    // (by cropping width in center)
    bool is_cropped;
    DepthFrame ml_depth_frame_cropped;    // in case the ml depth frame is cropped to the same size as the depth frame
    // Offset for cropping 
    // (original_width = cropping_width_offset_factor * original_width + cropped_width + cropping_width_offset_factor * original_width)
    double cropping_width_offset_factor;

    Frame();
    ~Frame();

    void preprocess_depth_frame();
    void preprocess_ml_depth_frame();
    bool crop_ml_depth_frame_if_needed();
    void preprocess_point_clouds();
    void preprocess();
};

#endif // FRAME_HPP