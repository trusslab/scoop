#include "PointCloudOpenCVBridge.hpp"

// Convert opencv depth frame to pcl point cloud (uint16)
void convert_depth_frame_to_pcl_point_cloud_uint16(DepthFrame &depth_frame)
{
    // Clear the point cloud
    depth_frame.point_cloud->clear();

    const CameraIntrinsics &camera_intrinsics = depth_frame.camera_intrinsics;
    const cv::Mat &depth_frame_raw = depth_frame.depth_frame_raw;
    PCLDepthFrameBiMap &pcl_cv_bi_map = depth_frame.pcl_cv_bi_map;

    // Loop through the depth frame and create the point cloud
    for (int i = 0; i < depth_frame_raw.rows; i++) {
        for (int j = 0; j < depth_frame_raw.cols; j++) {
            uint16_t depth = depth_frame_raw.at<uint16_t>(i, j);
            if (depth == 0) {
                continue;
            }
            pcl::PointXYZ point;
            point.z = (float)depth;
            point.x = (j - camera_intrinsics.cx) * point.z / camera_intrinsics.fx;
            point.y = (i - camera_intrinsics.cy) * point.z / camera_intrinsics.fy;
            depth_frame.point_cloud->push_back(point);
            pcl_cv_bi_map.insert(depth_frame.point_cloud->size() - 1, std::make_pair(j, i));
        }
    }
}

// Convert opencv depth frame to pcl point cloud (float16)
void convert_depth_frame_to_pcl_point_cloud_float(DepthFrame &depth_frame)
{
    // Clear the point cloud
    depth_frame.point_cloud->clear();

    const CameraIntrinsics &camera_intrinsics = depth_frame.camera_intrinsics;
    const cv::Mat &depth_frame_raw = depth_frame.depth_frame_raw;
    PCLDepthFrameBiMap &pcl_cv_bi_map = depth_frame.pcl_cv_bi_map;

    // Loop through the depth frame and create the point cloud
    for (int i = 0; i < depth_frame_raw.rows; i++) {
        for (int j = 0; j < depth_frame_raw.cols; j++) {
            float depth = depth_frame_raw.at<float>(i, j);
            if (depth == 0.0f || std::isnan(depth)) {
                continue;
            }
            pcl::PointXYZ point;
            point.z = depth;
            point.x = (j - camera_intrinsics.cx) * point.z / camera_intrinsics.fx;
            point.y = (i - camera_intrinsics.cy) * point.z / camera_intrinsics.fy;
            depth_frame.point_cloud->push_back(point);
            pcl_cv_bi_map.insert(depth_frame.point_cloud->size() - 1, std::make_pair(j, i));
        }
    }
}