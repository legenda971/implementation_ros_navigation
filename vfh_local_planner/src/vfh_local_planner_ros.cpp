#include <vfh_local_planner/vfh_local_planner_ros.h>

#include <pluginlib/class_list_macros.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(vfh_local_planner::VFHPlannerROS, nav_core::BaseLocalPlanner)

namespace vfh_local_planner
{
  void VFHPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (initialized_)
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }

    ROS_INFO("Initialize VFH Planer ROS.");
    initialized_ = true;
    // costmap_ros_ = costmap_ros;

    ROS_INFO("Name: %s", name.c_str());

    ros::NodeHandle pn("~/" + name);
    marker_pub_ = pn.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    pn.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
    pn.param("nearlest_point_number", nearlest_point_number_, 3);
    pn.param("nearlest_point_distance", look_ahead_distance_, 0.25);

    pn.param("angle_weight", angle_weight_, 1.0);
    pn.param("distance_weight", distance_weight_, 1.0);
    pn.param("delta_velocity_weight", delta_velocity_weight_, 1.0);

    pn.param("max_linear_velocity", max_linear_velocity_, 0.5);
    pn.param("max_angular_velocity", max_angular_velocity_, 0.5);

    pn.param("num_linear_velocities", num_linear_velocities_, 5);
    pn.param("num_angular_velocities", num_angular_velocities_, 5);

    pn.param("delta_t", delta_t_, 0.25);

    ROS_INFO("xy_goal_tolerance: %f", xy_goal_tolerance_);
  }

  bool VFHPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    global_plan_ = plan;
    ROS_INFO("Set new plan.");

    return true;
  }

  bool VFHPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // ROS_INFO("Compute velocity commands.");

    // vfh_local_planner::Polar_histogram polar_histogram(25, costmap_ros_);
    // polar_histogram.calculate_magnitude();
    // polar_histogram.drawHistogram(marker_pub_);
    cmd_vel.linear.x = 0.5;
    if (true)
      return true;

    // Find the target point in the global plan at the look-ahead distance from the robot's current position
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);
    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::min(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) / 2.0;

    // Find the target point in the global plan at the look-ahead distance from the robot's current position
    size_t target_point_index = 0;
    double min_distance_to_look_ahead = std::numeric_limits<double>::max();
    double robot_to_goal_x = global_plan_.back().pose.position.x - current_pose.pose.position.x;
    double robot_to_goal_y = global_plan_.back().pose.position.y - current_pose.pose.position.y;
    for (size_t i = 0; i < global_plan_.size(); ++i)
    {
      double distance_to_robot = std::hypot(global_plan_[i].pose.position.x - current_pose.pose.position.x,
                                            global_plan_[i].pose.position.y - current_pose.pose.position.y);

      // Check if the target point is within the distance threshold, just for filtering out the points that are too far away
      if (distance_to_robot > dist_threshold)
        continue;      

      double distance_to_look_ahead = std::abs(distance_to_robot - look_ahead_distance_);

      // Calculate the dot product of the vectors
      double robot_to_target_x = global_plan_[i].pose.position.x - current_pose.pose.position.x;
      double robot_to_target_y = global_plan_[i].pose.position.y - current_pose.pose.position.y;
      double dot_product = robot_to_target_x * robot_to_goal_x + robot_to_target_y * robot_to_goal_y;
      // Check if the dot product is positive and update the target point index if the distance to look-ahead is smaller
      if (dot_product > 0 && distance_to_look_ahead < min_distance_to_look_ahead)
      {
        min_distance_to_look_ahead = distance_to_look_ahead;
        target_point_index = i;
      }
    }
    ////

    // Generate candidate velocities (linear and angular)
    std::vector<std::pair<double, double>> candidate_velocities;

    // Define the step size for linear and angular velocities
    const double linear_step = max_linear_velocity_ / (num_linear_velocities_ - 1);
    const double angular_step = 2 * max_angular_velocity_ / (num_angular_velocities_ - 1);

    // Generate candidate velocities
    for (int i = 0; i < num_linear_velocities_; ++i)
    {
      double linear_velocity = i * linear_step;
      for (int j = 0; j < num_angular_velocities_; ++j)
      {
        double angular_velocity = -max_angular_velocity_ + j * angular_step;
        candidate_velocities.push_back({linear_velocity, angular_velocity});
      }
    }

    double target_angle = std::atan2(global_plan_[target_point_index].pose.position.y - current_pose.pose.position.y,
                                     global_plan_[target_point_index].pose.position.x - current_pose.pose.position.x);

    double current_distance_to_goal = std::hypot(current_pose.pose.position.x - global_plan_[target_point_index].pose.position.x,
                                                 current_pose.pose.position.y - global_plan_[target_point_index].pose.position.y);

    double progress = prev_distance_to_goal_ - current_distance_to_goal;
    prev_distance_to_goal_ = current_distance_to_goal;

    // Adjust the weights based on the robot's progress
    double progress_threshold = 0.1;
    if (progress < progress_threshold)
    {
      angle_weight_ *= 1.1;
      distance_weight_ *= 1.1;
      delta_velocity_weight_ *= 0.9;
    }
    else
    {
      angle_weight_ *= 0.9;
      distance_weight_ *= 0.9;
      delta_velocity_weight_ *= 1.1;
    }
    // Ensure that the weights are within a reasonable range
    angle_weight_ = std::max(0.1, std::min(angle_weight_, 10.0));
    distance_weight_ = std::max(0.1, std::min(distance_weight_, 10.0));
    delta_velocity_weight_ = std::max(0.1, std::min(delta_velocity_weight_, 10.0));

    ROS_INFO("angle_weight: %f, distance_weight: %f, delta_velocity_weight: %f", angle_weight_, distance_weight_, delta_velocity_weight_);

    // Evaluate candidate velocities
    double best_v = 0.0;
    double best_w = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();
    double current_angle = tf::getYaw(current_pose.pose.orientation);
    double max_linear_position = max_linear_velocity_ * delta_t_;
    for (const auto &candidate : candidate_velocities)
    {
      double v = candidate.first;
      double w = candidate.second;

      // Penalize distances to the global path with candidate velocities
      double angle = std::abs(angles::shortest_angular_distance(current_angle + w * delta_t_, target_angle) / M_PI);
      double x = current_pose.pose.position.x + v * delta_t_ * std::cos(current_angle + w * delta_t_);
      double y = current_pose.pose.position.y + v * delta_t_ * std::sin(current_angle + w * delta_t_);
      double distance_penalize = std::hypot(x - global_plan_[target_point_index].pose.position.x,
                                            y - global_plan_[target_point_index].pose.position.y) / max_linear_position;

      // Reward smoothness (minimize abrupt changes in velocities)
      double delta_v = std::abs(v - prev_cmd_vel_.linear.x) / max_linear_velocity_;
      double delta_w = std::abs(w - prev_cmd_vel_.angular.z) / max_angular_velocity_;
      double delta_velocity_penalize = std::sqrt(std::pow(delta_v, 2) + std::pow(delta_w, 2));

      // Each value is normalized to the range [0, 1]
      double score = -angle_weight_ * angle - distance_weight_ * distance_penalize - delta_velocity_weight_ * delta_velocity_penalize;
      // ROS_INFO("Angle: %f, distance: %f, delta_velocity: %f, score: %f, v %f, w %f", angle, distance_penalize, delta_velocity_penalize, score, v, w);
      if (score > best_score)
      {
        best_score = score;
        best_v = v;
        best_w = w;
      }
    }
    // ROS_INFO("Best v: %f, w: %f, score %f", best_v, best_w, best_score);

    // Create the Twist message
    cmd_vel.linear.x = best_v;
    cmd_vel.angular.z = best_w;

    // Store the current command for the next iteration
    prev_cmd_vel_ = cmd_vel;

    // Mark the target point in the global plan as a red sphere
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "target_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = global_plan_[target_point_index].pose.position.x;
    marker.pose.position.y = global_plan_[target_point_index].pose.position.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub_.publish(marker);
    //

    return true;
  }

  bool VFHPlannerROS::isGoalReached()
  {
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    if (global_plan_.empty())
    {
      ROS_ERROR("Received an empty plan.");
      return false;
    }

    // Get the current robot position and orientation
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);

    // Get the goal position and orientation
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    // Calculate the position error
    // double position_error = std::hypot(goal_pose.pose.position.x - current_pose.pose.position.x,
    //                                    goal_pose.pose.position.y - current_pose.pose.position.y);

    // if (position_error < xy_goal_tolerance_)
    // {
    //   ROS_INFO("Goal reached.");
    //   return true;
    // }

    return false;
  }
}