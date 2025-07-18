include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser",
  published_frame = "laser",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.2,  -- Faster publishing for Express mode
  pose_publish_period_sec = 4e-3,   -- Higher frequency pose updates
  trajectory_publish_period_sec = 20e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- RPLidar S2 Express mode optimized settings
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.12
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- Express mode specific optimizations
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150  -- More data for Express mode's higher frequency
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- Finer filtering for high-frequency scans
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)

-- Adaptive voxel filtering for high-frequency scans
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30.0

-- Loop closure optimization for Express mode
POSE_GRAPH.optimization_problem.huber_scale = 3e1
POSE_GRAPH.optimize_every_n_nodes = 50  -- More frequent optimization
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.62
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.0  -- Larger search area

return options 