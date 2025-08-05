#include "PointCloudHelpers.hpp"

// Normal estimation with k search
void pcl_normal_estimation_with_k_search(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, const int k_search_value)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(point_cloud);
    normal_estimator.setKSearch(k_search_value);
    normal_estimator.compute(*normals);
}

// Normal estimation with radius search
void pcl_normal_estimation_with_radius_search(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, const float radius_search_value)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(point_cloud);
    normal_estimator.setRadiusSearch(radius_search_value);
    normal_estimator.compute(*normals);

    // Do it with CUDA
    // pcl::cuda::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    // normal_estimator.setInputCloud(point_cloud);
    // normal_estimator.setRadiusSearch(radius_search_value);
    // normal_estimator.compute(*normals);
}

// Region growing segmentation
void region_growing_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const RegionGrowingConfig &region_growing_config, pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &reg, std::vector<pcl::PointIndices> &cluster_indices)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(point_cloud);
    normal_estimator.setKSearch(region_growing_config.ksearch);
    normal_estimator.compute(*normals);

    region_growing_segmentation(point_cloud, region_growing_config, reg, cluster_indices, normals);
}
void region_growing_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const RegionGrowingConfig &region_growing_config, pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &reg, std::vector<pcl::PointIndices> &cluster_indices, const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Region growing
    reg.setMinClusterSize(region_growing_config.min_cluster_size);
    reg.setMaxClusterSize(region_growing_config.max_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(region_growing_config.number_of_neighbours);
    reg.setInputCloud(point_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(region_growing_config.smoothness_threshold);
    reg.setCurvatureThreshold(region_growing_config.curvature_threshold);

    // Extract clusters
    reg.extract(cluster_indices);
}

// Caculate similarity between two pcl point clouds using Iterative Closest Point (ICP) Algorithm (with default ICPConfig)
std::tuple<bool, double> calculate_similarity_between_two_pcl_point_clouds_using_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2)
{
    ICPConfig icp_config;
    return calculate_similarity_between_two_pcl_point_clouds_using_icp(point_cloud1, point_cloud2, icp_config);
}

// Caculate similarity between two pcl point clouds using Iterative Closest Point (ICP) Algorithm
std::tuple<bool, double> calculate_similarity_between_two_pcl_point_clouds_using_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2, const ICPConfig &icp_config)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(icp_config.max_iterations);
    icp.setTransformationEpsilon(icp_config.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(icp_config.euclidean_fitness_epsilon);
    icp.setMaxCorrespondenceDistance(icp_config.max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold(icp_config.ransac_outlier_rejection_threshold);
    icp.setInputSource(point_cloud1);
    icp.setInputTarget(point_cloud2);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);

#ifdef ENABLE_DEBUG_LOG
    std::cout << "Converged: " << icp.hasConverged() << " score: " << (icp.getFitnessScore() / point_cloud1->size()) << std::endl;
#ifdef ENABLE_DEBUG_LOG_VERBOSE
    std::cout << icp.getFinalTransformation() << std::endl;
#endif  // ENABLE_DEBUG_LOG_VERBOSE
#endif  // ENABLE_DEBUG_LOG

    return std::make_tuple(icp.hasConverged(), icp.getFitnessScore());
}

// Function to find correspondences between two sets of FPFH features and compute an average similarity score
double find_correspondences_and_compute_similarity(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs1, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs2)
{
    // KdTree for searching FPFH features
    pcl::KdTreeFLANN<pcl::FPFHSignature33> matcher;
    matcher.setInputCloud(fpfhs2); // Use fpfhs2 as the reference

    // Variables to hold sum of distances and count for averaging
    double totalDistance = 0.0;
    int count = 0;

    // Iterate over each feature in fpfhs1, find its nearest neighbor in fpfhs2
    for (size_t i = 0; i < fpfhs1->size(); ++i) {
        std::vector<int> nearestIndices(1); // To hold the index of the nearest neighbor
        std::vector<float> nearestDistances(1); // To hold the squared distance to the nearest neighbor

        if (!std::isfinite(fpfhs1->points[i].histogram[0])) { // Skip invalid descriptors
            continue;
        }

        // Find the nearest neighbor
        int foundNeighbors = matcher.nearestKSearch(fpfhs1->points[i], 1, nearestIndices, nearestDistances);
        if (foundNeighbors == 1) {
            totalDistance += sqrt(nearestDistances[0]); // Use sqrt to convert squared distance to actual distance
            ++count;
        }
    }

    if (count == 0)
        return std::numeric_limits<double>::max(); // Handle case with no valid correspondences

    double averageDistance = totalDistance / count;

    std::cout << "Average distance between corresponding FPFH features: " << averageDistance << " with count: " << count << std::endl;

    return averageDistance; // Lower values indicate more similar point clouds
}

// Caculate similarity between two pcl point clouds using Fast Point Feature Histograms (FPFH) Algorithm
double calculate_similarity_between_two_pcl_point_clouds_using_fpfh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
    // pcl_normal_estimation_with_k_search(point_cloud1, normals1, 50);
    pcl_normal_estimation_with_radius_search(point_cloud1, normals1);
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
    // pcl_normal_estimation_with_k_search(point_cloud2, normals2, 50);
    pcl_normal_estimation_with_radius_search(point_cloud2, normals2);

    // Compute FPFH features
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1 (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs2 (new pcl::PointCloud<pcl::FPFHSignature33> ());
    
    fpfh.setInputCloud (point_cloud1);
    fpfh.setInputNormals (normals1);
    fpfh.setSearchMethod (tree);
    fpfh.setRadiusSearch (0.05);
    fpfh.compute (*fpfhs1);
    
    fpfh.setInputCloud (point_cloud2);
    fpfh.setInputNormals (normals2);
    fpfh.compute (*fpfhs2);
    return find_correspondences_and_compute_similarity(fpfhs1, fpfhs2);
}

// Function to compute SHOT features
void compute_shot_features(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::SHOT352>::Ptr &shot_features, const float radius_search_value)
{
#ifdef PCL_OMP
    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
#else
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
#endif  // PCL_OMP
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    shot.setSearchMethod(tree);
    shot.setInputCloud(point_cloud);
    shot.setInputNormals(normals);
    shot.setRadiusSearch(radius_search_value);
    shot.compute(*shot_features);
}

// Cacularate similarity between two pcl point clouds using SHOT descriptors
#include <pcl/filters/filter.h>
double calculate_similarity_between_two_pcl_point_clouds_using_shot(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud2)
{
    // Remove invalid points
    // pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud1_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // std::vector<int> mapping;
    // pcl::removeNaNFromPointCloud(*point_cloud1, *point_cloud1_filtered, mapping);

    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
    // pcl_normal_estimation_with_k_search(point_cloud1, normals1, 50);
    pcl_normal_estimation_with_radius_search(point_cloud1, normals1);
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
    // pcl_normal_estimation_with_k_search(point_cloud2, normals2, 50);
    pcl_normal_estimation_with_radius_search(point_cloud2, normals2);

    // Compute SHOT features
    pcl::PointCloud<pcl::SHOT352>::Ptr shot_features1(new pcl::PointCloud<pcl::SHOT352>);
    compute_shot_features(point_cloud1, normals1, shot_features1, 0.05f);
    // compute_shot_features(point_cloud1, normals1, shot_features1, 0.15f);
    pcl::PointCloud<pcl::SHOT352>::Ptr shot_features2(new pcl::PointCloud<pcl::SHOT352>);
    compute_shot_features(point_cloud2, normals2, shot_features2, 0.05f);
    // compute_shot_features(point_cloud2, normals2, shot_features2, 0.15f);

    // Find correspondences between SHOT features and compute an average similarity score
    pcl::Correspondences correspondences;
    pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
    est.setInputSource(shot_features1); // SHOT from first cloud
    est.setInputTarget(shot_features2); // SHOT from second cloud
    est.determineCorrespondences(correspondences);

    double totalDistance = 0.0;
    for (const auto& corr : correspondences) {
        totalDistance += corr.distance;
    }
    double averageDistance = totalDistance / correspondences.size();
    std::cout << "Average descriptor distance: " << averageDistance << std::endl;

    return averageDistance;
    // return 0.0;
}

//  Calculate similarity between two point clouds using Clustered Viewpoint Feature Histogram (CVFH) descriptors
#include <pcl/features/cvfh.h>
double calculate_similarity_between_two_pcl_point_clouds_using_cvfh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) 
{
    // Setup objects
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    
    // Compute normals for both clouds
    pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
    pcl_normal_estimation_with_radius_search(cloud1, normals1);
    pcl_normal_estimation_with_radius_search(cloud2, normals2);
    
    // Setup CVFH estimation
    pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setSearchMethod(kdtree);
    
    // Set parameters for CVFH computation
    cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI);  // 5 degrees
    cvfh.setCurvatureThreshold(1.0);
    cvfh.setNormalizeBins(true);
    
    // Compute CVFH features for first cloud
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfh1(new pcl::PointCloud<pcl::VFHSignature308>);
    cvfh.setInputCloud(cloud1);
    cvfh.setInputNormals(normals1);
    cvfh.compute(*cvfh1);
    
    // Compute CVFH features for second cloud
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfh2(new pcl::PointCloud<pcl::VFHSignature308>);
    cvfh.setInputCloud(cloud2);
    cvfh.setInputNormals(normals2);
    cvfh.compute(*cvfh2);
    
    // Compare histograms using Chi-Square distance
    double similarity = 0.0;
    for (int i = 0; i < 308; ++i) {
        double diff = cvfh1->points[0].histogram[i] - cvfh2->points[0].histogram[i];
        similarity += diff * diff;
    }
    similarity = sqrt(similarity);
    
    return similarity;
}
