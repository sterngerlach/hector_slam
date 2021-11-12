
# ScanMatcherGaussNewtonMetrics.msg

# Metrics updated in HectorMappingRos::scanMatchCallback()
# Timestamp of the scan matching start
uint64 stamp
# Grid map resolution level
int32 map_resolution_level

# Metrics updated in ScanMatcherGaussNewton::MatchScans()
# Time for the pose optimization
duration optimization_time
# Distance between the initial pose estimate and the final pose estimate
float32 diff_translation
# Absolute difference between the initial angle and the final angle estimate
float32 diff_rotation
# Total number of the iterations
int32 num_of_iterations
# Initial correspondence cost
float32 initial_correspondence_cost
# Final correspondence cost
float32 final_correspondence_cost
# Number of the points in the given scan
int32 num_of_scans
