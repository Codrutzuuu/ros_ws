global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

explore:
  ros__parameters:
    robot_base_frame: base_link
    costmap_topic: /map  
    odom_topic: /odometry/filtered
    scan_topic: /scan
    explore_frequency: 1.0
    planner_frequency: 0.5
    min_frontier_size: 0.5
    progress_timeout: 30.0
    visualize: true

ekf_filter_node:
  ros__parameters:
    transform_time_offset: 0.1
    transform_timeout: 0.0
    two_d_mode: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /odom
    odom0_config: [true, true, false, false, false, true, false, false, false, false, false, false, false, false, false]
    odom0_differential: false
    imu0: /imu/data
    imu0_config: [false, false, false, true, true, true, false, false, false, true, true, true, false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true