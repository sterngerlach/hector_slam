
# ScanMatcherCorrelativeMetrics.msg

# Metrics updated in HectorMappingRos::scanMatchCallback()
# Timestamp of the scan matching start
uint64 stamp
# Grid map resolution level
int32 map_resolution_level

# Metrics updated in ScanMatcherCorrelative::MatchScans()
# Time for the pose optimization
duration optimization_time
# Distance between the initial pose estimate and the final pose estimate
float32 diff_translation
# Absolute difference between the initial angle and the final angle estimate
float32 diff_rotation
# Size of the search window (translation)
int32 win_size_x
# Size of the search window (translation)
int32 win_size_y
# Size of the search window (rotation)
int32 win_size_theta
# Step size (translation)
float32 step_size_x
# Step size (translation)
float32 step_size_y
# Step size (rotation)
float32 step_size_theta
# Number of the skipped nodes
int32 num_of_skipped_nodes
# Number of the processed nodes
int32 num_of_processed_nodes
# Normalized score for the best solution
float32 score
# Number of the points in the given scan
int32 num_of_scans
