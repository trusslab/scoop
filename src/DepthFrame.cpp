#include "DepthFrame.hpp"
#include "PointCloudOpenCVBridge.hpp"

// Camera intrinsics class constructor
CameraIntrinsics::CameraIntrinsics() {}

// Camera intrinsics class constructor
CameraIntrinsics::CameraIntrinsics(const float& fx, const float& fy, const float& cx, const float& cy)
{
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
}

// Camera intrinsics class destructor
CameraIntrinsics::~CameraIntrinsics() {}

// Depth frame class constructor
DepthFrame::DepthFrame() { }

// Depth frame class constructor
DepthFrame::DepthFrame(const CameraIntrinsics& camera_intrinsics)
{
    this->camera_intrinsics = camera_intrinsics;
}

// Depth frame class destructor
DepthFrame::~DepthFrame() {}

// Prepare normalized depth frame
void DepthFrame::prepare_normalized_depth_frame()
{
    cv::normalize(depth_frame_raw, depth_frame_raw_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}

// Prepare normalized inverted depth frame
void DepthFrame::prepare_normalized_inverted_depth_frame()
{
    cv::bitwise_not(depth_frame_raw_normalized, depth_frame_raw_normalized_inverted);
}

// Prepare point cloud
void DepthFrame::prepare_point_cloud()
{
    this->pcl_cv_bi_map.clear();
    
    point_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (this->depth_frame_raw.type() == CV_16U)
    {
        convert_depth_frame_to_pcl_point_cloud_uint16(*this);
    }
    else if (this->depth_frame_raw.type() == CV_32F)
    {
        convert_depth_frame_to_pcl_point_cloud_float(*this);
    }
    else
    {
        std::cerr << "Depth frame type not supported" << std::endl;
        exit(EXIT_FAILURE);
    }
}

#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE && defined ENABLE_DEBUG_LOG_VERBOSE_AVG_DEPTH
// Print average depth
void DepthFrame::print_average_depth()
{
    double average_depth = 0.0f;
    uint32_t total_points = 0;
    for (int i = 0; i < depth_frame_raw.rows; i++) {
        for (int j = 0; j < depth_frame_raw.cols; j++) {
            float depth = depth_frame_raw.at<float>(i, j);
            if (depth != 0.0f && !std::isnan(depth)) {
                average_depth += depth;
                total_points++;
            }
        }
    }
    average_depth /= total_points;
    std::cout << "Average depth in the frame: " << average_depth << " with total points: " << total_points << std::endl;
}
#endif
