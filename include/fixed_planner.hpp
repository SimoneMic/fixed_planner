#ifndef FIXED_PLANNER_HPP_
#define FIXED_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace fixed_planner{

class WaypointInterpolatedPlanner : public nav2_core::GlobalPlanner
{
public:
    WaypointInterpolatedPlanner() = default;
    ~WaypointInterpolatedPlanner() = default;

    // plugin configuration
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // Method that creates the path starting from the point configured in the yaml file
    // linking all the waypoints passed from the config
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal
    ) override;

private:

    std::shared_ptr<tf2_ros::Buffer> m_tf;
    nav2_util::LifecycleNode::SharedPtr m_node;
    nav2_costmap_2d::Costmap2D * m_costmap;
    std::string m_global_frame;
    std::string m_name;
    double m_interpolation_resolution, m_rotational_interpolation_resolution;

    std::vector<std::vector<double>> m_waypoint_list;
    bool m_interpolate_waypoints{true};
};

}   //namespace fixed_planner

#endif //FIXED_PLANNER_HPP_