#ifndef DEPTH_FRAME_HPP
#define DEPTH_FRAME_HPP

#include "CompilationFlags.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>

#include "PointCloudBase.hpp"

// Camera intrinsics class
class CameraIntrinsics
{
    public:
    float fx; // in pixels
    float fy; // in pixels
    float cx; // in pixels
    float cy; // in pixels

    CameraIntrinsics();
    CameraIntrinsics(const float& fx, const float& fy, const float& cx, const float& cy);

    ~CameraIntrinsics();
};

// Depth frame class
class DepthFrame
{
    public:
    CameraIntrinsics camera_intrinsics;
    cv::Mat depth_frame_raw;
    cv::Mat depth_frame_raw_normalized;
    cv::Mat depth_frame_raw_normalized_inverted;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
    PCLDepthFrameBiMap pcl_cv_bi_map;   // Map between Point Cloud and Depth Frame (Index and Pair of X, Y coordinates)

    DepthFrame();
    DepthFrame(const CameraIntrinsics& camera_intrinsics);
    ~DepthFrame();

    void prepare_normalized_depth_frame();
    void prepare_normalized_inverted_depth_frame();
    void prepare_point_cloud();

#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE && defined ENABLE_DEBUG_LOG_VERBOSE_AVG_DEPTH
    void print_average_depth();
#endif
};

#endif // DEPTH_FRAME_HPP