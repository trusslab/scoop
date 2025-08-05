#include "Analyzer.hpp"

#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iterator>
#include <vector>
#include <cstdint>
#include <thread>
#include <utility>

// OpenCV includes
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp> // Gaussian Blur
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/ximgproc.hpp>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Frame includes
#include "Frame.hpp"

// Default values
#include "CameraParams.hpp"

// Other includes
#include "Utils.hpp"
#include "ConfigParser.hpp"
#include "Config.hpp"

// Depth includes
#include "DepthFrame.hpp"
#include "DepthMediaFile.hpp"

// Media includes
#include "Media.hpp"

// Point cloud helpers
#include "PointCloudHelpers.hpp"

// Indicator for progress bar
#include <indicators/progress_bar.hpp>

// Parameters set automatically
bool excution_platform_endianness = 0; // endianness of the platform that this program executes on; 0 means little, 1 means big (TODO: implement big endian support)

// Main function
int main (int argc, char *argv[])
{
    // Special checks
    excution_platform_endianness = is_current_system_big_endian();
    if (excution_platform_endianness != 0) {
        std::cout << "TODO: implement big endian support" << std::endl;
        return -1;
    }

    // Init configuration
    AnalyzerConfig analyzer_config(argc, argv);

    // Open media files
    Media media(analyzer_config);

    // TODO: Add progress bar
    // indicators::ProgressBar global_pbar;

    // Main loop
    Frame frame;
    uint64_t frame_counter = 0;
    int res;
    while (true) {

        // Get next frame
        res = media.get_next_frame(frame);
        if (res) {
            break;
        }

#if defined ENABLE_HEURISTIC_OVERALL_ICP
        auto [overall_icp_converged, overall_icp_similarity] = calculate_similarity_between_two_pcl_point_clouds_using_icp(frame.depth_frame.point_cloud, frame.ml_depth_frame->point_cloud, analyzer_config.pre_whole_frame_icp_config);
#if defined ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
        std::cout << "[HEURISTIC_OVERALL_ICP]: ICP similarity: " << overall_icp_similarity << std::endl;
#endif // ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
        if (overall_icp_converged && overall_icp_similarity < analyzer_config.global_icp_similarity_threshold)
            continue;
#endif // ENABLE_HEURISTIC_OVERALL_ICP

#if defined ENABLE_HEURISTIC_OVERALL_CVFH
        auto overall_cvfh_similarity = calculate_similarity_between_two_pcl_point_clouds_using_cvfh(frame.depth_frame.point_cloud, frame.ml_depth_frame->point_cloud);
#if defined ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
        std::cout << "[HEURISTIC_OVERALL_CVFH]: Overall CVFH similarity: " << overall_cvfh_similarity << std::endl;
#endif // ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
        if (overall_cvfh_similarity < analyzer_config.global_cvfh_similarity_threshold)
            continue;
#endif // ENABLE_HEURISTIC_OVERALL_CVFH

        // Read config file again (if dynamic config is enabled)
#ifdef ENABLE_DYNAMIC_CONFIG
        analyzer_config.read_config();
#endif

        // Find surfaces in the pcl point cloud (Region Growing Algorithm)
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        std::vector<pcl::PointIndices> ground_truth_cloud_clusters;
        region_growing_segmentation(frame.depth_frame.point_cloud, analyzer_config.depth_pcl_region_growing_segmentation_config, reg, ground_truth_cloud_clusters);

        // DEBUG: print number of clusters
#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE
        std::cout << "Number of clusters: " << ground_truth_cloud_clusters.size() << std::endl;
#endif

        // NOTE: Maybe plane segmentation can also be used.

        // Find surfaces in the ml pcl point cloud (Region Growing Algorithm)
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> ml_reg;
        std::vector<pcl::PointIndices> ml_clusters;
        region_growing_segmentation(frame.ml_depth_frame->point_cloud, analyzer_config.ml_depth_pcl_region_growing_segmentation_config, ml_reg, ml_clusters);

        // DEBUG: print number of clusters
#if defined ENABLE_DEBUG_LOG && defined ENABLE_DEBUG_LOG_VERBOSE
        std::cout << "Number of ml clusters: " << ml_clusters.size() << std::endl;
#endif

        // Init Violation Visualization frame
        cv::Mat violation_frame = cv::Mat::zeros(frame.depth_frame.depth_frame_raw.rows, frame.depth_frame.depth_frame_raw.cols, CV_8UC3);

        // Violation related variables
        bool is_frame_suspicious = false;
        long unsigned int num_of_suspicious_ml_clusters = 0;

#if defined DEBUG_VISUALIZE_GROUND_TRUTH_AND_ML_DEPTH_FRAME_IN_SAME_WINDOW
        // DEBUG: Visualize both point clouds
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(frame.depth_frame.point_cloud, "depth cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "depth cloud");
        viewer->addPointCloud<pcl::PointXYZ>(frame.ml_depth_frame->point_cloud, "ml cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.99, 0.81, 0.01, "ml cloud");
        viewer->spin();
#endif // DEBUG_VISUALIZE_GROUND_TRUTH_AND_ML_DEPTH_FRAME_IN_SAME_WINDOW

        // Loop through each cluster found in the depth frame
        for (int i = 0; i < (int)(ground_truth_cloud_clusters.size()); i++) {
            // Use the cluster to mask the ml point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr masked_ml_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (int j = 0; j < (int)(ground_truth_cloud_clusters[i].indices.size()); j++) {
                int depth_cloud_index = ground_truth_cloud_clusters[i].indices[j];
                std::pair<int, int> depth_coordinates = frame.depth_frame.pcl_cv_bi_map.get_coordinates(depth_cloud_index);
                if (!frame.ml_depth_frame->pcl_cv_bi_map.contains(depth_coordinates)) {
                    continue;
                }
                masked_ml_cloud->push_back(frame.ml_depth_frame->point_cloud->points[frame.ml_depth_frame->pcl_cv_bi_map.get_index(depth_coordinates)]);
            }

            // Remove outliers from the ml pcl point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr ml_depth_frame_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(masked_ml_cloud);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(*ml_depth_frame_cloud_filtered);

            // Apply region growing segmentation to the masked ml point cloud
            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> masked_ml_reg;
            std::vector<pcl::PointIndices> masked_ml_cloud_clusters;
            region_growing_segmentation(ml_depth_frame_cloud_filtered, analyzer_config.ml_depth_pcl_region_growing_segmentation_config, masked_ml_reg, masked_ml_cloud_clusters);

            // Detect violation
            bool is_violation = (masked_ml_cloud_clusters.size() > 1);

            // Perform heuristics
            if (is_violation) {

                // Init for heuristics
#if defined ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE || defined ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE
                uint32_t num_of_largest_masked_ml_cluster_points = 0;
                uint32_t num_of_second_largest_masked_ml_cluster_points = 0;
                uint32_t total_num_of_masked_ml_cluster_points;
#endif // ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE || ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE

#if defined ENABLE_HEURISTIC_SUB_CLOUD_PERCENTAGE
                if (ground_truth_cloud_clusters[0].indices.size() < (size_t)(analyzer_config.heuristic_sub_cloud_percentage_threshold * frame.depth_frame.point_cloud->size())) {
#if defined ENABLE_DEBUG_LOG_HEURISTICS
                    std::cout << "[HEURISTIC_SUB_CLOUD_PERCENTAGE]: Violation canceled with sub_cloud size(%): ";
                    std::cout << (double)(ground_truth_cloud_clusters[0].indices.size()) / (double)(frame.depth_frame.point_cloud->size());
                    std::cout << "(< " << analyzer_config.heuristic_sub_cloud_percentage_threshold * 100 << "%)" << std::endl;
#endif // ENABLE_DEBUG_LOG_HEURISTICS
                    is_violation = false;
                    goto end_of_heuristics;
                }
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_PERCENTAGE

#if defined ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE || defined ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE
                for (int j = 0; j < (int)(masked_ml_cloud_clusters.size()); j++) {
                    // Accumulate total number of points
                    total_num_of_masked_ml_cluster_points += masked_ml_cloud_clusters[j].indices.size();

                    // Find the two largest clusters
                    if (masked_ml_cloud_clusters[j].indices.size() > num_of_largest_masked_ml_cluster_points) {
                        num_of_second_largest_masked_ml_cluster_points = num_of_largest_masked_ml_cluster_points;
                        num_of_largest_masked_ml_cluster_points = masked_ml_cloud_clusters[j].indices.size();
                    } else if (masked_ml_cloud_clusters[j].indices.size() > num_of_second_largest_masked_ml_cluster_points) {
                        num_of_second_largest_masked_ml_cluster_points = masked_ml_cloud_clusters[j].indices.size();
                    }
                }
#endif  // ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE || ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE

#ifdef ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE
                if (is_violation) {
                    if (((float)num_of_largest_masked_ml_cluster_points) / ((float)total_num_of_masked_ml_cluster_points) >= analyzer_config.heuristic_sub_cluster_percentage_threshold) {
#if defined ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
                        std::cout << "[HEURISTIC_SUB_CLUSTER_PERCENTAGE]: Violation canceled with max occupancy: ";
                        std::cout << ((float)num_of_largest_masked_ml_cluster_points) / ((float)total_num_of_masked_ml_cluster_points);
                        std::cout << "(max: " << num_of_largest_masked_ml_cluster_points << ", " << "total: ";
                        std::cout << total_num_of_masked_ml_cluster_points << ")" << std::endl;
#endif  // ENABLE_DEBUG_LOG && ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
                        is_violation = false;
                        goto end_of_heuristics;
                    }
                }
#endif  // ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE

#ifdef ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE
                if (is_violation) {
                    if (((float)num_of_largest_masked_ml_cluster_points + (float)num_of_second_largest_masked_ml_cluster_points) / ((float)total_num_of_masked_ml_cluster_points) >= analyzer_config.heuristic_two_sub_clusters_percentage_threshold) {
#if defined ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
                        std::cout << "[HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE]: Violation canceled with max occupancy: ";
                        std::cout << ((float)num_of_largest_masked_ml_cluster_points + (float)num_of_second_largest_masked_ml_cluster_points) / ((float)total_num_of_masked_ml_cluster_points);
                        std::cout << "(max: " << num_of_largest_masked_ml_cluster_points << ", ";
                        std::cout << num_of_second_largest_masked_ml_cluster_points << ", total: ";
                        std::cout << total_num_of_masked_ml_cluster_points << ")" << std::endl;
#endif  // ENABLE_DEBUG_LOG && ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE
                        is_violation = false;
                        goto end_of_heuristics;
                    }
                }
#endif  // ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE

                // If violation is still detected, check for similarity
                if (is_violation) {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_sub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr ml_depth_sub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for (int j = 0; j < (int)(ground_truth_cloud_clusters[i].indices.size()); j++) {
                        int depth_cloud_index = ground_truth_cloud_clusters[i].indices[j];
                        std::pair<int, int> depth_coordinates = frame.depth_frame.pcl_cv_bi_map.get_coordinates(depth_cloud_index);

                        depth_sub_cloud->push_back(frame.depth_frame.point_cloud->points[depth_cloud_index]);
                        if (frame.ml_depth_frame->pcl_cv_bi_map.contains(depth_coordinates)) {
                            ml_depth_sub_cloud->push_back(frame.ml_depth_frame->point_cloud->points[frame.ml_depth_frame->pcl_cv_bi_map.get_index(depth_coordinates)]);
                        }
                    }

#if defined ENABLE_HEURISTIC_SUB_CLOUD_ICP
                    auto [icp_converged, icp_similarity] = calculate_similarity_between_two_pcl_point_clouds_using_icp(depth_sub_cloud, ml_depth_sub_cloud, analyzer_config.sub_frame_icp_config);
                    if (icp_converged && icp_similarity < analyzer_config.global_icp_similarity_threshold) {

#if defined ENABLE_HEURISTIC_SUB_CLOUD_CVFH
                        // Calculate and print average and median of the two point clouds
                        double depth_sub_cloud_avg = 0.0;
                        double ml_depth_sub_cloud_avg = 0.0;
                        double depth_sub_cloud_median = 0.0;
                        double ml_depth_sub_cloud_median = 0.0;
                        std::vector<double> depth_sub_cloud_z_values;
                        std::vector<double> ml_depth_sub_cloud_z_values;
                        for (int j = 0; j < (int)(depth_sub_cloud->size()); j++) {
                            depth_sub_cloud_avg += depth_sub_cloud->at(j).z;
                            ml_depth_sub_cloud_avg += ml_depth_sub_cloud->at(j).z;

                            depth_sub_cloud_z_values.push_back(depth_sub_cloud->at(j).z);
                            ml_depth_sub_cloud_z_values.push_back(ml_depth_sub_cloud->at(j).z);
                        }
                        depth_sub_cloud_avg /= depth_sub_cloud->size();
                        ml_depth_sub_cloud_avg /= ml_depth_sub_cloud->size();
                        depth_sub_cloud_median = compute_median(depth_sub_cloud_z_values);
                        ml_depth_sub_cloud_median = compute_median(ml_depth_sub_cloud_z_values);
                        std::cout << "Depth sub cloud avg: " << depth_sub_cloud_avg << "; median: " << depth_sub_cloud_median << std::endl;
                        std::cout << "ML Depth sub cloud avg: " << ml_depth_sub_cloud_avg << "; median: " << ml_depth_sub_cloud_median << std::endl;

                        // Calculate and print mean squared error of the two point clouds
                        double mse = 0.0;
                        for (int j = 0; j < (int)(depth_sub_cloud->size()); j++) {
                            mse += pow(depth_sub_cloud->at(j).z - ml_depth_sub_cloud->at(j).z, 2);
                        }
                        mse /= depth_sub_cloud->size();
                        std::cout << "Mean Squared Error(Exp.): " << mse << std::endl;

                        // Calculate and print mean squared error of each of the two point clouds
                        double depth_sub_cloud_mse = 0.0;
                        double ml_depth_sub_cloud_mse = 0.0;
                        double sub_cloud_mse_diff = 0.0;
                        for (int j = 0; j < (int)(depth_sub_cloud->size()); j++) {
                            depth_sub_cloud_mse += pow(depth_sub_cloud->at(j).z - depth_sub_cloud_avg, 2);
                            ml_depth_sub_cloud_mse += pow(ml_depth_sub_cloud->at(j).z - ml_depth_sub_cloud_avg, 2);
                        }
                        depth_sub_cloud_mse /= depth_sub_cloud->size();
                        ml_depth_sub_cloud_mse /= ml_depth_sub_cloud->size();
                        std::cout << "Depth sub cloud MSE: " << depth_sub_cloud_mse << std::endl;
                        std::cout << "ML Depth sub cloud MSE: " << ml_depth_sub_cloud_mse << std::endl;
                        if (ml_depth_sub_cloud_mse > depth_sub_cloud_mse)
                            sub_cloud_mse_diff = ml_depth_sub_cloud_mse - depth_sub_cloud_mse;
                        std::cout << "sub_cloud_mse_diff: " << sub_cloud_mse_diff << "; threshold: " << analyzer_config.heuristic_three_mse_diff_threshold << std::endl;

                        if (mse < analyzer_config.heuristic_three_mse_threshold && sub_cloud_mse_diff < analyzer_config.heuristic_three_mse_diff_threshold) {
                            is_violation = false;
                            goto end_of_heuristics;
                        }
                        
                        auto sub_cvfh_similarity = calculate_similarity_between_two_pcl_point_clouds_using_cvfh(frame.depth_frame.point_cloud, frame.ml_depth_frame->point_cloud);
                        if (sub_cvfh_similarity < analyzer_config.global_cvfh_similarity_threshold) {
                            is_violation = false;
                            goto end_of_heuristics;
                        }
                        else {
                            std::cout << "(Violation Detected)Sub CVFH similarity: " << sub_cvfh_similarity << " v.s. " << analyzer_config.global_cvfh_similarity_threshold << std::endl;
                            goto end_of_heuristics;
                        }
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_CVFH
                        is_violation = false;
                        goto end_of_heuristics;
                    }
                    else
                        std::cout << "(Violation Detected)ICP similarity: " << icp_similarity << std::endl;
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_ICP
                }
            }

            end_of_heuristics:

#if defined DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD || defined DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD
            // DEBUG: Visualize violation in 3D
            if (is_violation) {
                // DEBUG: Visualize the depth sub cloud in its main depth cloud with a different color
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_sub_colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                for (int j = 0; j < (int)(ground_truth_cloud_clusters[i].indices.size()); j++) {
                    int depth_cloud_index = ground_truth_cloud_clusters[i].indices[j];
                    pcl::PointXYZ original_point = frame.depth_frame.point_cloud->at(depth_cloud_index);
                    pcl::PointXYZRGB point;
                    point.x = original_point.x;
                    point.y = original_point.y;
                    point.z = original_point.z;
                    point.r = 0;
                    point.g = 255;
                    point.b = 0;
                    depth_sub_colored_cloud->push_back(point);
                }
#if defined DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD
                pcl::visualization::PCLVisualizer viewer("PCL Viewer");
                viewer.setBackgroundColor(0, 0, 0);
                viewer.addPointCloud(frame.depth_frame.point_cloud, "depth cloud");
                viewer.addPointCloud(depth_sub_colored_cloud, "depth sub cloud");
                viewer.spin();
#endif // DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD

#if defined DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD
                // DEBUG: Visualize the ml depth sub cloud in its main depth cloud with a different color
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr ml_depth_sub_colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                for (int j = 0; j < (int)(ground_truth_cloud_clusters[i].indices.size()); j++) {
                    int depth_cloud_index = ground_truth_cloud_clusters[i].indices[j];
                    std::pair<int, int> depth_coordinates = frame.depth_frame.pcl_cv_bi_map.get_coordinates(depth_cloud_index);
                    if (!frame.ml_depth_frame->pcl_cv_bi_map.contains(depth_coordinates)) {
                        continue;
                    }
                    pcl::PointXYZ original_point = frame.ml_depth_frame->point_cloud->at(frame.ml_depth_frame->pcl_cv_bi_map.get_index(depth_coordinates));
                    pcl::PointXYZRGB point;
                    point.x = original_point.x;
                    point.y = original_point.y;
                    point.z = original_point.z;
                    point.r = 252;
                    point.g = 207;
                    point.b = 3;
                    ml_depth_sub_colored_cloud->push_back(point);
                }
                pcl::visualization::PCLVisualizer ml_viewer("PCL ML Viewer");
                ml_viewer.setBackgroundColor(0, 0, 0);
                // ml_viewer.addPointCloud(frame.depth_frame.point_cloud, "depth cloud");
                ml_viewer.addPointCloud(depth_sub_colored_cloud, "depth sub cloud");
                // ml_viewer.addPointCloud(frame.ml_depth_frame->point_cloud, "ml depth cloud");
                ml_viewer.addPointCloud(ml_depth_sub_colored_cloud, "ml depth sub cloud");
                ml_viewer.spin();
#endif // DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD
            }
#endif // DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD || DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD

            // Mark the violation points in the violation frame
            if (is_violation) {
                is_frame_suspicious = true;
                num_of_suspicious_ml_clusters = masked_ml_cloud_clusters.size();

                // Visualize violation in 2D
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr violation_colored_cloud = ml_reg.getColoredCloud();
                for (int j = 0; j < (int)(ground_truth_cloud_clusters[i].indices.size()); j++) {
                    int depth_cloud_index = ground_truth_cloud_clusters[i].indices[j];
                    std::pair<int, int> depth_coordinates = frame.depth_frame.pcl_cv_bi_map.get_coordinates(depth_cloud_index);
                    if (!frame.ml_depth_frame->pcl_cv_bi_map.contains(depth_coordinates)) {
                        continue;
                    }
                    int p_j = depth_coordinates.first;
                    int p_i = depth_coordinates.second;
                    pcl::PointXYZRGB point = violation_colored_cloud->at(frame.ml_depth_frame->pcl_cv_bi_map.get_index(depth_coordinates));
                    violation_frame.at<cv::Vec3b>(p_i, p_j)[0] = point.b;
                    violation_frame.at<cv::Vec3b>(p_i, p_j)[1] = point.g;
                    violation_frame.at<cv::Vec3b>(p_i, p_j)[2] = point.r;
                }

                break;
            }
        }

        // Visualize surface clusters
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
        cv::Mat depth_frame_clusters = cv::Mat::zeros(frame.depth_frame.depth_frame_raw.rows, frame.depth_frame.depth_frame_raw.cols, CV_8UC3);
        for (int i = 0; i < depth_frame_clusters.rows; i++) {
            for (int j = 0; j < depth_frame_clusters.cols; j++) {
                std::pair<int, int> depth_coordinates = std::make_pair(j, i);
                if (!frame.depth_frame.pcl_cv_bi_map.contains(depth_coordinates)) {
                    continue;
                }
                pcl::PointXYZRGB point = colored_cloud->at(frame.depth_frame.pcl_cv_bi_map.get_index(depth_coordinates));
                depth_frame_clusters.at<cv::Vec3b>(i, j)[0] = point.b;
                depth_frame_clusters.at<cv::Vec3b>(i, j)[1] = point.g;
                depth_frame_clusters.at<cv::Vec3b>(i, j)[2] = point.r;
            }
        }

        // Visualize ml surface clusters
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ml_colored_cloud = ml_reg.getColoredCloud();
        cv::Mat ml_depth_frame_clusters = cv::Mat::zeros(frame.ml_depth_frame->depth_frame_raw.rows, frame.ml_depth_frame->depth_frame_raw.cols, CV_8UC3);
        for (int i = 0; i < ml_depth_frame_clusters.rows; i++) {
            for (int j = 0; j < ml_depth_frame_clusters.cols; j++) {
                std::pair<int, int> depth_coordinates = std::make_pair(j, i);
                if (!frame.ml_depth_frame->pcl_cv_bi_map.contains(depth_coordinates)) {
                    continue;
                }
                pcl::PointXYZRGB point = ml_colored_cloud->at(frame.ml_depth_frame->pcl_cv_bi_map.get_index(depth_coordinates));
                ml_depth_frame_clusters.at<cv::Vec3b>(i, j)[0] = point.b;
                ml_depth_frame_clusters.at<cv::Vec3b>(i, j)[1] = point.g;
                ml_depth_frame_clusters.at<cv::Vec3b>(i, j)[2] = point.r;
            }
        }

        // Visualize violation in RGB frame by highlighting the violation points
        // first scale the violation frame to the size of the rgb frame
        cv::Mat violation_frame_resized;
        cv::resize(violation_frame, violation_frame_resized, cv::Size(frame.rgb_frame.cols, frame.rgb_frame.rows));
        cv::Mat rgb_frame_violation = frame.rgb_frame.clone();
        for (int i = 0; i < rgb_frame_violation.rows; i++) {
            for (int j = 0; j < rgb_frame_violation.cols; j++) {
                if (violation_frame_resized.at<cv::Vec3b>(i, j)[0] != 0 || violation_frame_resized.at<cv::Vec3b>(i, j)[1] != 0 || violation_frame_resized.at<cv::Vec3b>(i, j)[2] != 0) {
                    rgb_frame_violation.at<cv::Vec3b>(i, j)[0] = 0;
                    rgb_frame_violation.at<cv::Vec3b>(i, j)[1] = 0;
                    rgb_frame_violation.at<cv::Vec3b>(i, j)[2] = 210;
                }
            }
        }

        if (is_frame_suspicious) {
            printf("Violation detected: %s with size of masked_ml_cloud_clusters: %lu\n", is_frame_suspicious ? "true" : "false", num_of_suspicious_ml_clusters);

#if defined DISPLAY_VIOLATED_FRAME
            // Resize and display frames
            cv::Mat rgb_frame_resized;
            cv::Mat depth_frame_resized;
            cv::Mat surface_frame_first_resized;
            cv::Mat surface_clusters_resized;
            cv::Mat ml_surface_clusters_resized;
            cv::Mat ml_depth_frame_resized;
            cv::resize(frame.rgb_frame, rgb_frame_resized, cv::Size(std::round(float(frame.rgb_frame.cols) * float(DISPLAY_WINDOW_HEIGHT) / float(frame.rgb_frame.rows)), DISPLAY_WINDOW_HEIGHT));
            cv::resize(frame.depth_frame.depth_frame_raw_normalized_inverted, depth_frame_resized, cv::Size(std::round(float(frame.depth_frame.depth_frame_raw_normalized_inverted.cols) * float(DISPLAY_WINDOW_HEIGHT) / float(frame.depth_frame.depth_frame_raw_normalized_inverted.rows)), DISPLAY_WINDOW_HEIGHT));
            cv::resize(violation_frame, surface_frame_first_resized, cv::Size(std::round(float(violation_frame.cols) * float(DISPLAY_WINDOW_HEIGHT) / float(violation_frame.rows)), DISPLAY_WINDOW_HEIGHT));
            cv::resize(depth_frame_clusters, surface_clusters_resized, cv::Size(std::round(float(depth_frame_clusters.cols) * float(DISPLAY_WINDOW_HEIGHT) / float(depth_frame_clusters.rows)), DISPLAY_WINDOW_HEIGHT));
            cv::resize(ml_depth_frame_clusters, ml_surface_clusters_resized, cv::Size(std::round(float(ml_depth_frame_clusters.cols) * float(DISPLAY_WINDOW_HEIGHT) / float(ml_depth_frame_clusters.rows)), DISPLAY_WINDOW_HEIGHT));
            cv::resize(frame.ml_depth_frame->depth_frame_raw_normalized, ml_depth_frame_resized, cv::Size(std::round(float(frame.ml_depth_frame->depth_frame_raw_normalized.cols) * float(DISPLAY_WINDOW_HEIGHT) / float(frame.ml_depth_frame->depth_frame_raw_normalized.rows)), DISPLAY_WINDOW_HEIGHT));
            cv::imshow("RGB", rgb_frame_resized);
            cv::imshow("Depth", depth_frame_resized);
            cv::imshow("Surface Violation", surface_frame_first_resized);
            cv::imshow("Surface Clusters", surface_clusters_resized);
            cv::imshow("ML Surface Clusters", ml_surface_clusters_resized);
            cv::imshow("ML Depth", ml_depth_frame_resized);

            // Resize and display rgb frame with violation
            cv::Mat rgb_frame_violation_resized;
            cv::resize(rgb_frame_violation, rgb_frame_violation_resized, cv::Size(std::round(float(rgb_frame_violation.cols) * float(DISPLAY_WINDOW_HEIGHT) / float(rgb_frame_violation.rows)), DISPLAY_WINDOW_HEIGHT));
            cv::imshow("RGB Violation", rgb_frame_violation_resized);

            // Wait for rgb_frame_interval milliseconds (TODO: use media.get_rgb_frame_interval() instead)
            // if (cv::waitKey(media.get_rgb_frame_interval()) == 27) {
            //     std::cout << "User pressed ESC key. Exiting..." << std::endl;
            //     break;
            // }
            if (cv::waitKey(0) == 27) {
                std::cout << "User pressed ESC key. Exiting..." << std::endl;
                break;
            }
#endif // DISPLAY_VIOLATED_FRAME
        }
        
        frame_counter++;
    }

    return 0;
}