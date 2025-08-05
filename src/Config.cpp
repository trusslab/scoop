#include "Config.hpp"

RegionGrowingConfig::RegionGrowingConfig()
{
    reset();
}

RegionGrowingConfig::RegionGrowingConfig(ConfigParser &config_parser, const std::string section_label)
{
    read_config(config_parser, section_label);
}

void RegionGrowingConfig::read_config(ConfigParser &config_parser, const std::string section_label)
{
    ksearch = config_parser.aConfig<uint32_t>(section_label, "ksearch");
    min_cluster_size = config_parser.aConfig<uint32_t>(section_label, "min_cluster_size");
    max_cluster_size = config_parser.aConfig<uint32_t>(section_label, "max_cluster_size");
    number_of_neighbours = config_parser.aConfig<uint32_t>(section_label, "number_of_neighbours");
    smoothness_threshold = config_parser.aConfig<float>(section_label, "smoothness_threshold");
    if (smoothness_threshold == 0.0) {
        smoothness_threshold = 3.0 / 180.0 * M_PI;
    }
    curvature_threshold = config_parser.aConfig<float>(section_label, "curvature_threshold");
}

// Set with default values
void RegionGrowingConfig::reset()
{
    ksearch = 50;
    min_cluster_size = 50;
    max_cluster_size = 1000000;
    number_of_neighbours = 30;
    smoothness_threshold = 3.0 / 180.0 * M_PI;
    curvature_threshold = 1.0;
}

ICPConfig::ICPConfig()
{
    reset();
}

ICPConfig::ICPConfig(ConfigParser &config_parser, const std::string section_label)
{
    read_config(config_parser, section_label);
}

void ICPConfig::read_config(ConfigParser &config_parser, const std::string section_label)
{
    max_iterations = config_parser.aConfig<uint32_t>(section_label, "max_iterations");
    transformation_epsilon = config_parser.aConfig<double>(section_label, "transformation_epsilon");
    euclidean_fitness_epsilon = config_parser.aConfig<double>(section_label, "euclidean_fitness_epsilon");
    max_correspondence_distance = config_parser.aConfig<double>(section_label, "max_correspondence_distance");
    ransac_outlier_rejection_threshold = config_parser.aConfig<double>(section_label, "ransac_outlier_rejection_threshold");
}

// Set with default values
void ICPConfig::reset()
{
    max_iterations = 50;
    transformation_epsilon = 1e-8;
    euclidean_fitness_epsilon = 1;
    max_correspondence_distance = 0.05; // 1cm
    ransac_outlier_rejection_threshold = 0.03;
}

AnalyzerConfig::AnalyzerConfig(const int argc, char *argv[])
{
    switch (argc) {
        case 5:
        {
            this->is_processing_rgb_video = (std::stoi(argv[1]) == 1);
            std::string abbrev_fname = std::string(argv[2]);
            std::string platform_id = std::string(argv[3]);
            if (this->is_processing_rgb_video) {
                this->rgb_media_file_name = abbrev_fname + "_rgb.mp4";
            } else {
                this->rgb_media_file_name = abbrev_fname + "_rgb.jpg";
            }
            if (platform_id == "0") {
                this->depth_media_file_name = abbrev_fname + "_depth.adep";
            } else if (platform_id == "1") {
                this->depth_media_file_name = abbrev_fname + "_depth.idep";
            }
            this->ml_depth_media_file_name = abbrev_fname + "_rgb_depth_metric.ml_depth_pro.mldep";
            this->config_parser = ConfigParser(std::string(argv[4]));
            break;
        }
        case 6:
        {
            this->is_processing_rgb_video = (std::stoi(argv[1]) == 1);
            this->rgb_media_file_name = std::string(argv[2]);
            this->depth_media_file_name = std::string(argv[3]);
            this->ml_depth_media_file_name = std::string(argv[4]);
            this->config_parser = ConfigParser(std::string(argv[5]));
            break;
        }
        default:
            std::cout << "Usage: ./analyzer [IS_VIDEO(0/1)] [RGB_VIDEO_FILE] [DEPTH_VIDEO_FILE] [ML_DEPTH_VIDEO_FILE] [CONFIG]" << std::endl;
            std::cout << "Usage: ./analyzer [IS_VIDEO(0/1)] [FILE_ABBREV] [PLATFORM_ID] [CONFIG]" << std::endl;
            exit(1);
    }
    
    read_config();
}

void AnalyzerConfig::read_config()
{
    // Read application level configurations
#if defined ENABLE_HEURISTIC_OVERALL_ICP || defined ENABLE_HEURISTIC_SUB_CLOUD_ICP
    global_icp_similarity_threshold = config_parser.aConfig<double>(APPLICATION_PARAMS, "global_icp_similarity_threshold");
#endif  // ENABLE_HEURISTIC_OVERALL_ICP
#if defined ENABLE_HEURISTIC_OVERALL_CVFH || defined ENABLE_HEURISTIC_SUB_CLOUD_CVFH
    global_cvfh_similarity_threshold = config_parser.aConfig<double>(APPLICATION_PARAMS, "global_cvfh_similarity_threshold");
#endif  // ENABLE_HEURISTIC_OVERALL_CVFH
#if defined ENABLE_HEURISTIC_SUB_CLOUD_PERCENTAGE
    heuristic_sub_cloud_percentage_threshold = config_parser.aConfig<double>(APPLICATION_PARAMS, "heuristic_sub_cloud_percentage_threshold");
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_PERCENTAGE
#if defined ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE
    heuristic_sub_cluster_percentage_threshold = config_parser.aConfig<double>(APPLICATION_PARAMS, "heuristic_sub_cluster_percentage_threshold");
#endif  // ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE
#if defined ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE
    heuristic_two_sub_clusters_percentage_threshold = config_parser.aConfig<double>(APPLICATION_PARAMS, "heuristic_two_sub_clusters_percentage_threshold");
#endif  // ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE
#if defined ENABLE_HEURISTIC_SUB_CLOUD_CVFH
    heuristic_three_mse_threshold = config_parser.aConfig<double>(APPLICATION_PARAMS, "heuristic_three_mse_threshold");
    heuristic_three_mse_diff_threshold = config_parser.aConfig<double>(APPLICATION_PARAMS, "heuristic_three_mse_diff_threshold");
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_CVFH

    // Read algorithm level configurations
    read_depth_pcl_region_growing_segmentation_config(DEPTH_FRAME_PCL_CLUSTERING_PARAMS);
    read_ml_depth_pcl_region_growing_segmentation_config(ML_DEPTH_FRAME_PCL_CLUSTERING_PARAMS);
    
#if defined ENABLE_HEURISTIC_OVERALL_ICP
    pre_whole_frame_icp_config.read_config(config_parser, PRE_WHOLE_FRAME_ICP_COMPARISON_PARAMS);
#endif  // ENABLE_HEURISTIC_OVERALL_ICP
#if defined ENABLE_HEURISTIC_SUB_CLOUD_ICP
    sub_frame_icp_config.read_config(config_parser, SUB_FRAME_ICP_COMPARISON_PARAMS);
#endif  // ENABLE_HEURISTIC_SUB_CLOUD_ICP
}

void AnalyzerConfig::read_depth_pcl_region_growing_segmentation_config(const std::string &section_label)
{
    depth_pcl_region_growing_segmentation_config.read_config(config_parser, section_label);
}

void AnalyzerConfig::read_ml_depth_pcl_region_growing_segmentation_config(const std::string &section_label)
{
    ml_depth_pcl_region_growing_segmentation_config.read_config(config_parser, section_label);
}


