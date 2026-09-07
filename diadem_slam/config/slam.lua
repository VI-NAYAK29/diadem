include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  -- Cartographer publishes the full map->odom->base_link tree
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  -- Predict from the recent scan-matched velocity between scans.
  use_pose_extrapolator = true,
  -- Standalone scan-matching mode: use external odom as prior
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  -- Publish submaps at 10 Hz so costmap and constraints are always fresh
  submap_publish_period_sec = 0.1,
  -- Publish pose at 500 Hz to minimise TF latency for Nav2
  pose_publish_period_sec = 2e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.50
TRAJECTORY_BUILDER_2D.max_range = 25.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 30.
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- ─── Real-time Correlative Scan Matcher ────────────────────────────────────
-- Wide window: at >1 m/s the robot can shift 30+ cm between match cycles;
-- the correlative pre-matcher must cover that entire range or Ceres starts
-- from a bad initial guess and drift is never recovered.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.30
-- Near-zero cost weights: let the correlative matcher try ALL candidate
-- translations/rotations freely — Ceres does the fine-grained refinement.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.01
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight    = 0.01

-- ─── Ceres Scan Matcher ─────────────────────────────────────────────────────
-- High occupied_space_weight: force the solver to snap scan points onto
-- existing submap cells.  This is the primary knob for instantaneous
-- translational snap-back on a fast robot.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 35.
-- Very low translation/rotation weights: Ceres will not resist large pose
-- corrections — it always defers to where the LiDAR points actually land.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight    = 10.

-- ─── Motion Filter ──────────────────────────────────────────────────────────
-- At 1 m/s+ the robot covers 2 cm in 20 ms.  Time-gate at 20 ms so a
-- correction fires on virtually every LiDAR sweep regardless of linear speed.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = math.rad(0.05)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- ─── Submaps ────────────────────────────────────────────────────────────────
-- Smaller submaps (60 scans instead of default 100): loop-closure constraints
-- are generated much sooner after a region is revisited, correcting drift
-- before it accumulates.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- ─── Pose Graph ─────────────────────────────────────────────────────────────
-- Low min_score: any partial feature overlap generates a constraint.
POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
-- sampling_ratio = 1.0: check every candidate submap pair so no correction
-- opportunity is skipped.
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
POSE_GRAPH.optimization_problem.huber_scale = 1e2
-- Optimise after every single new node for instant graph correction.
POSE_GRAPH.optimize_every_n_nodes = 1

return options
