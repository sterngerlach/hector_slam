
# HectorMappingMetrics.msg

# Scan matcher option (synchronized with ScanMatcherOption enum)
int32 SCAN_MATCHER_UNKNOWN = 0
int32 SCAN_MATCHER_DEFAULT = 1
int32 SCAN_MATCHER_GAUSS_NEWTON = 2
int32 SCAN_MATCHER_CORRELATIVE = 3
int32 SCAN_MATCHER_CORRELATIVE_FPGA = 4
int32 SCAN_MATCHER_GAUSS_NEWTON_AFTER_CORRELATIVE = 5
int32 SCAN_MATCHER_GAUSS_NEWTON_AFTER_CORRELATIVE_FPGA = 6

# Timestamp of the message
# Updated in HectorMappingRos::scanCallback()
uint64 stamp
# Timestamp of the current scan
# Updated in HectorMappingRos::scanCallback()
uint64 scan_time_stamp

# Time for processing the current frame (scan)
# Updated in HectorMappingRos::scanCallback()
duration processing_time
# Time for preprocessing the current scan
# Updated in HectorMappingRos::scanCallback()
duration scan_preprocessing_time
# Time for the scan matching
# Updated in HectorSlamProcessor::update()
duration scan_matching_time
# Time for the map update
# Updated in HectorSlamProcessor::update()
duration map_update_time
# Time for publishing the results
# Updated in HectorMappingRos::scanCallback()
duration result_publish_time

# Flag to indicate whether the scan matching is skipped
# Updated in HectorSlamProcessor::update()
bool scan_matching_skipped
# Flag to indicate whether the grid map update is skipped
# Updated in HectorSlamProcessor::update()
bool map_update_skipped

# Initial pose estimate
# Updated in HectorSlamProcessor::update()
float32[3] initial_pose_estimate
# Final pose estimate
# Updated in HectorSlamProcessor::update()
float32[3] final_pose_estimate
# Pose covariance
# Updated in HectorSlamProcessor::update()
float32[9] pose_covariance

# Scan matcher option (ScanMatcherOption enum)
# Updated in HectorMappingRos::scanCallback()
int32 scan_matcher_option
# Metrics for Gauss-Newton based scan matcher
# Updated in ScanMatcherGaussNewton::MatchScans()
ScanMatcherGaussNewtonMetrics[] scan_matcher_gauss_newton_metrics
# Metrics for real-time correlative scan matcher
# Updated in ScanMatcherCorrelative::MatchScans()
ScanMatcherCorrelativeMetrics[] scan_matcher_correlative_metrics
# Metrics for FPGA-based real-time correlative scan matcher
# Updated in ScanMatcherFPGA::MatchScans()
ScanMatcherFPGAMetrics[] scan_matcher_fpga_metrics
