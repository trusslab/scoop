#ifndef POINT_CLOUD_OPENCV_BRIDGE_HPP
#define POINT_CLOUD_OPENCV_BRIDGE_HPP

#include "CompilationFlags.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>

#include "DepthFrame.hpp"

// Convert opencv depth frame to pcl point cloud (uint16)
void convert_depth_frame_to_pcl_point_cloud_uint16(DepthFrame &depth_frame);

// Convert opencv depth frame to pcl point cloud (float)
void convert_depth_frame_to_pcl_point_cloud_float(DepthFrame &depth_frame);

#endif // POINT_CLOUD_OPENCV_BRIDGE_HPP