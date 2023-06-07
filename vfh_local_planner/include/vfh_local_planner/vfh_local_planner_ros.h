#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <boost/shared_ptr.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vfh_local_planner/polar_histogram.h>

namespace vfh_local_planner
{
  class VFHPlannerROS : public nav_core::BaseLocalPlanner
  {
  public:
    VFHPlannerROS() : initialized_(false) {}
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
    bool isGoalReached();

  private:
    bool initialized_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    ros::Publisher marker_pub_;

    int nearlest_point_number_; // nearlest point in the global plan
    double look_ahead_distance_;

    double xy_goal_tolerance_; // xy goal tolerance

    // Weights for the cost function
    double angle_weight_;
    double distance_weight_;
    double delta_velocity_weight_;

    // From this parameters, we generate candidate velocities
    double max_linear_velocity_;  // m/s
    double max_angular_velocity_; // rad/s

    int num_linear_velocities_; // number of linear velocities
    int num_angular_velocities_; // number of angular velocities

    double delta_t_; // time interval look ahead for the candidate velocities

    geometry_msgs::Twist prev_cmd_vel_;
    double prev_distance_to_goal_ = std::numeric_limits<double>::infinity();
  };
};
#endif
