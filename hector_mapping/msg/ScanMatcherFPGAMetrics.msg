
# ScanMatcherFPGAMetrics.msg

# Metrics updated in HectorMappingRos::scanMatchCallback()
# Timestamp of the scan matching start
uint64 stamp
# Grid map resolution level
int32 map_resolution_level

# Metrics updated in ScanMatcherFPGA::MatchScans()
# Time for the pose optimization
duration optimization_time
# Time for setting up the input
duration input_setup_time
# Time for setting up the IP core
duration ip_setup_time
# Time for transferring scans
duration scan_transfer_time
# Time for transferring grid maps
duration map_transfer_time
# Time for the computation on the IP core
duration ip_optimization_time
# Time for waiting for the IP core
duration ip_wait_time
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
# Size of the transferred grid map
int32 transferred_map_size_x
# Size of the transferred grid map
int32 transferred_map_size_y
# Number of the transferred blocks in the grid map, where block is the
# occupancy probabilities in the eight consecutive grid cells
int32 num_of_transferred_map_blocks
# Normalized score for the best solution
float32 score
# Number of the points in the given scan
int32 num_of_scans
