#ifndef POINT_CLOUD_HELPERS_HPP
#define POINT_CLOUD_HELPERS_HPP

#include "CompilationFlags.hpp"

#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>

// CUDA support
// #ifdef PCL_CUDA
// #include <pcl/cuda/
// #endif  // PCL_CUDA

#ifdef PCL_OMP
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#else
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#endif  // PCL_OMP

#include <opencv2/core/core.hpp>

#include "Config.hpp"

// Normal estimation with k search
void pcl_normal_estimation_with_k_search(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, const int k_search_value = 50);

// Normal estimation with radius search
void pcl_normal_estimation_with_radius_search(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, const float radius_search_value = 0.03f);

// Region growing segmentation
void region_growing_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const RegionGrowingConfig &region_growing_config, pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &reg, std::vector<pcl::PointIndices> &cluster_indices);
void region_growing_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const RegionGrowingConfig &region_growing_config, pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &reg, std::vector<pcl::PointIndices> &cluster_indices, const pcl::PointCloud<pcl::Normal>::Ptr &normals);

// Caculate similarity between two pcl point clouds using Iterative Closest Point (ICP) Algorithm
std::tuple<bool, double> calculate_similarity_between_two_pcl_point_clouds_using_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2);
std::tuple<bool, double> calculate_similarity_between_two_pcl_point_clouds_using_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2, const ICPConfig &icp_config);

// Function to find correspondences between two sets of FPFH features and compute an average similarity score
double find_correspondences_and_compute_similarity(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs1, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs2);

// Caculate similarity between two pcl point clouds using Fast Point Feature Histograms (FPFH) Algorithm
double calculate_similarity_between_two_pcl_point_clouds_using_fpfh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2);

// Function to compute SHOT features
void compute_shot_features(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::SHOT352>::Ptr &shot_features, const float radius_search_value = 0.03f);

// Cacularate similarity between two pcl point clouds using SHOT descriptors
double calculate_similarity_between_two_pcl_point_clouds_using_shot(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2);

//  Calculate similarity between two point clouds using Clustered Viewpoint Feature Histogram (CVFH) descriptors
double calculate_similarity_between_two_pcl_point_clouds_using_cvfh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

#endif // POINT_CLOUD_HELPERS_HPP