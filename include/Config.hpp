#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cstdint>
#include <math.h>
#include <string>

#include "ConfigParser.hpp"
#include "CompilationFlags.hpp"

// Section labels
#define APPLICATION_PARAMS "application_params"
#define DEPTH_FRAME_PCL_CLUSTERING_PARAMS "depth_frame_pcl_clustering_params"
#define ML_DEPTH_FRAME_PCL_CLUSTERING_PARAMS "ml_depth_frame_pcl_clustering_params"
#define PRE_WHOLE_FRAME_ICP_COMPARISON_PARAMS "pre_whole_frame_icp_comparison_params"
#define SUB_FRAME_ICP_COMPARISON_PARAMS "sub_frame_icp_comparison_params"

class RegionGrowingConfig
{
    public:
    uint32_t ksearch;
    uint32_t min_cluster_size;
    uint32_t max_cluster_size;
    uint32_t number_of_neighbours;
    float smoothness_threshold;
    float curvature_threshold;

    RegionGrowingConfig();
    RegionGrowingConfig(ConfigParser &config_parser, const std::string section_label);

    void read_config(ConfigParser &config_parser, const std::string section_label);

    // Set with default values
    void reset();
};

class ICPConfig
{
    public:
    uint32_t max_iterations;
    double transformation_epsilon;
    double euclidean_fitness_epsilon;
    double max_correspondence_distance;
    double ransac_outlier_rejection_threshold;

    ICPConfig();
    ICPConfig(ConfigParser &config_parser, const std::string section_label);

    void read_config(ConfigParser &config_parser, const std::string section_label);

    // Set with default values
    void reset();
};

class AnalyzerConfig
{
    public:
    // Mode
    bool is_processing_rgb_video;

    // File names
    std::string rgb_media_file_name;
    std::string depth_media_file_name;
    std::string ml_depth_media_file_name;

    ConfigParser config_parser;

    // Application level configurations
#if defined ENABLE_HEURISTIC_OVERALL_ICP || defined ENABLE_HEURISTIC_SUB_CLOUD_ICP
    double global_icp_similarity_threshold;
#endif  // ENABLE_HEURISTIC_OVERALL_ICP
#if defined ENABLE_HEURISTIC_OVERALL_CVFH || defined ENABLE_HEURISTIC_SUB_CLOUD_CVFH
    double global_cvfh_similarity_threshold;
#endif  // ENABLE_HEURISTIC_OVERALL_CVFH
#if defined ENABLE_HEURISTIC_SUB_CLOUD_PERCENTAGE
    double heuristic_sub_cloud_percentage_threshold;
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_PERCENTAGE
#if defined ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE
    double heuristic_sub_cluster_percentage_threshold;
#endif  // ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE
#if defined ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE
    double heuristic_two_sub_clusters_percentage_threshold;
#endif  // ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE
#if defined ENABLE_HEURISTIC_SUB_CLOUD_CVFH
    double heuristic_three_mse_threshold;
    double heuristic_three_mse_diff_threshold;
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_CVFH

    // Algorithm level configurations
    RegionGrowingConfig depth_pcl_region_growing_segmentation_config;
    RegionGrowingConfig ml_depth_pcl_region_growing_segmentation_config;
    ICPConfig pre_whole_frame_icp_config;
    ICPConfig sub_frame_icp_config;

    AnalyzerConfig(const int argc, char *argv[]);

    void read_config();

    private:
    void read_depth_pcl_region_growing_segmentation_config(const std::string &section_label);
    void read_ml_depth_pcl_region_growing_segmentation_config(const std::string &section_label);
};

#endif // CONFIG_HPP