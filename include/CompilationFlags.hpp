#ifndef CompilationFlags_HPP
#define CompilationFlags_HPP

// Heuristic config
#define ENABLE_HEURISTICS  // Enable heuristics
#if defined ENABLE_HEURISTICS

// #define ENABLE_HEURISTIC_OVERALL_ICP  // Check if the overall(entire frame) ICP similarity is below a certain threshold
// #define ENABLE_HEURISTIC_OVERALL_CVFH  // Check if the overall(entire frame) CVFH similarity is below a certain threshold
#define ENABLE_HEURISTIC_MULTI_FRAME_TRIGGER  // Violation is triggered if the heuristic is violated for a certain number of frames
#define ENABLE_HEURISTIC_SUB_CLOUD_PERCENTAGE  // Check how much percentage is the sub_cloud of the whole cloud
#define ENABLE_HEURISTIC_SUB_CLUSTER_PERCENTAGE  // Check if the largest cluster has a certain percentage of the total points
#define ENABLE_HEURISTIC_TWO_SUB_CLUSTERS_PERCENTAGE  // Check if the largest cluster and the second largest cluster combined have a certain percentage of the total points
#define ENABLE_HEURISTIC_SUB_CLOUD_ICP  // Check if the ICP similarity between the sub_clouds(ground truth v.s. ML) is below a certain threshold
#define ENABLE_HEURISTIC_SUB_CLOUD_CVFH  // Check if the CVFH similarity between the sub_clouds(ground truth v.s. ML) is below a certain threshold

#endif  // ENABLE_HEURISTICS

// Enable low-res depth frame for iOS platform
// #define ENABLE_LOW_RES_DEPTH_FRAME_FOR_IOS_PLATFORM  // Enable low-res depth frame for iOS platform (320x180) instead of the full resolution (768x432)

// Display violated frame
// #define DISPLAY_VIOLATED_FRAME  // Display the violated frame with highlighted points in 2D

// DEBUG: Visualize the violated ground truth sub cloud in the ground truth point cloud
// #define DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD

// DEBUG: Visualize the violated ML sub cloud in the ground truth point cloud
// #define DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD

// DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD and DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD should not be defined at the same time
#if defined DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD && defined DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD
#error "DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD and DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD should not be defined at the same time"
#endif // DEBUG_VISUALIZE_VIOLATED_GROUND_TRUTH_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD && DEBUG_VISUALIZE_VIOLATED_ML_SUB_CLOUD_IN_GROUND_TRUTH_POINT_CLOUD

// DEBUG: Visualize both ground truth and ML depth frames in the same window
// #define DEBUG_VISUALIZE_GROUND_TRUTH_AND_ML_DEPTH_FRAME_IN_SAME_WINDOW

// Debug log (Main; Cat 0)
// #define ENABLE_DEBUG_LOG
#if defined ENABLE_DEBUG_LOG

// Debug log (Cat 1)
#define ENABLE_DEBUG_LOG_VERBOSE
#if defined ENABLE_DEBUG_LOG_VERBOSE

#define ENABLE_DEBUG_LOG_VERBOSE_AVG_DEPTH
// #define ENABLE_DEBUG_LOG_VERBOSE_DEPTH_FRAME_CAMERA_INTRINSICS
// #define ENABLE_DEBUG_LOG_VERBOSE_ML_DEPTH_FRAME_CROPPING
#define ENABLE_DEBUG_LOG_VERBOSE_POINT_CLOUD_PREPROCESSING

#endif // ENABLE_DEBUG_LOG_VERBOSE

// Debug log (Cat 2)
#define ENABLE_DEBUG_LOG_HEURISTICS
#if defined ENABLE_DEBUG_LOG_HEURISTICS

#define ENABLE_DEBUG_LOG_HEURISTICS_VERBOSE

#endif // ENABLE_DEBUG_LOG_HEURISTICS

#endif // ENABLE_DEBUG_LOG

// Debug config
// #define ENABLE_DYNAMIC_CONFIG    // read config file every frame
// #define ENABLE_PCL_OMP  // enable OpenMP for PCL (TODO: Implement this)

#endif // CompilationFlags_HPP