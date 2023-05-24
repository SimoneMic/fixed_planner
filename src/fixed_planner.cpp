#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "fixed_planner.hpp"

namespace fixed_planner{

void WaypointInterpolatedPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
)
{
    m_node = parent.lock();
    m_name = name;
    m_tf = tf;
    m_costmap = costmap_ros->getCostmap();
    m_global_frame = costmap_ros->getGlobalFrameID();

    nav2_util::declare_parameter_if_not_declared(
        m_node, 
        m_name + ".interpolation_resolution", 
        rclcpp::ParameterValue(0.1)
        );
    m_node->get_parameter(m_name + ".interpolation_resolution", m_interpolation_resolution);

    nav2_util::declare_parameter_if_not_declared(
        m_node, 
        m_name + ".rotational_interpolation_resolution", 
        rclcpp::ParameterValue(M_PI/18)
        );
    m_node->get_parameter(m_name + ".rotational_interpolation_resolution", m_rotational_interpolation_resolution);

    nav2_util::declare_parameter_if_not_declared(
        m_node, 
        m_name + ".interpolate_waypoints", 
        rclcpp::ParameterValue(false)
        );
    m_node->get_parameter(m_name + ".interpolate_waypoints", m_interpolate_waypoints);

    //nav2_util::declare_parameter_if_not_declared(
    //    m_node, 
    //    m_name + ".x_displacement_list", 
    //    rclcpp::ParameterValue(std::vector<double>{0.0})
    //    );
    //m_node->get_parameter(m_name + ".x_displacement_list", m_x_displacement_list);

    //nav2_util::declare_parameter_if_not_declared(
    //    m_node, 
    //    m_name + ".y_displacement_list", 
    //    rclcpp::ParameterValue(std::vector<double>{0.0})
    //    );
    //m_node->get_parameter(m_name + ".y_displacement_list", m_y_displacement_list);

    //nav2_util::declare_parameter_if_not_declared(
    //    m_node, 
    //    m_name + ".theta_displacement_list", 
    //    rclcpp::ParameterValue(std::vector<double>{0.0})
    //    );
    //m_node->get_parameter(m_name + ".theta_displacement_list", m_theta_displacement_list);

    nav2_util::declare_parameter_if_not_declared(
        m_node, 
        m_name + ".zero_pose", 
        rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0})
        );
    m_node->get_parameter(m_name + ".zero_pose", m_zero_pose);
    
    // TODO - do parameters checks

    // TODO - Remove and use parameters instead
    double zero_x, zero_y, zero_theta;
    zero_x = m_zero_pose[0];
    zero_y = m_zero_pose[1];
    zero_theta = m_zero_pose[2];
    m_waypoint_list.push_back({zero_x, zero_y, zero_theta});     // x, y, theta
    m_waypoint_list.push_back({zero_x + 0.0, zero_y + 1.0, zero_theta + 0.0});      // 0 1 0
    m_waypoint_list.push_back({zero_x + 0.5, zero_y + 1.0, zero_theta + 0.0});      // 0.5 1 0
    m_waypoint_list.push_back({zero_x + 0.5, zero_y - 1.0, zero_theta + 0.0});
    m_waypoint_list.push_back({zero_x + 1.0, zero_y - 1.0, zero_theta + 0.0});
    m_waypoint_list.push_back({zero_x + 1.5, zero_y + 0.0, zero_theta + 0.0});
    m_waypoint_list.push_back({zero_x + 1.0, zero_y + 0.5, zero_theta + M_PI_4});
    m_waypoint_list.push_back({zero_x + 0.5, zero_y + 0.0, zero_theta + M_PI_4});
    m_waypoint_list.push_back({zero_x + 0.0, zero_y + 0.0, zero_theta + M_PI});
}

void WaypointInterpolatedPlanner::cleanup()
{
  RCLCPP_INFO(
    m_node->get_logger(), "CleaningUp plugin %s",
    m_name.c_str());
}

void WaypointInterpolatedPlanner::activate()
{
  RCLCPP_INFO(
    m_node->get_logger(), "Activating plugin %s",
    m_name.c_str());
}

void WaypointInterpolatedPlanner::deactivate()
{
  RCLCPP_INFO(
    m_node->get_logger(), "Deactivating plugin %s",
    m_name.c_str());
}

nav_msgs::msg::Path WaypointInterpolatedPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
)
{
    //TODO - initial checks
    nav_msgs::msg::Path global_path;
    global_path.poses.clear();
    global_path.header.stamp = m_node->now();
    global_path.header.frame_id = m_global_frame;

    if (m_interpolate_waypoints)
    {
        // size - 1 iterations
        for (size_t i = 1; i < m_waypoint_list.size(); ++i)
        {
            //number of iterations to move frome start to goal
            int number_of_loop_translation = std::hypot(
                m_waypoint_list[i][0] - m_waypoint_list[i-1][0],
                m_waypoint_list[i][1] - m_waypoint_list[i-1][1]
            ) / m_interpolation_resolution;
            int number_of_loop_rotation = std::abs(m_waypoint_list[i][2] - m_waypoint_list[i-1][2]) / m_rotational_interpolation_resolution;

            double x_increment = (m_waypoint_list[i][0] - m_waypoint_list[i-1][0]) / number_of_loop_translation;
            double y_increment = (m_waypoint_list[i][1] - m_waypoint_list[i-1][1]) / number_of_loop_translation;
            double theta_increment = (m_waypoint_list[i][2] - m_waypoint_list[i-1][2]) / number_of_loop_rotation;

            //Interpolation
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = m_global_frame;
            pose.pose.position.z = 0.0;
            int count = 0;
            if (number_of_loop_translation >= number_of_loop_rotation)
            {
                for (int j = 0; j < number_of_loop_translation; ++j)
                {
                    pose.header.stamp = m_node->now();
                    pose.pose.position.x = m_waypoint_list[i-1][0] + x_increment * j;
                    pose.pose.position.y = m_waypoint_list[i-1][1] + y_increment * j;

                    double angle = m_waypoint_list[i-1][2] + theta_increment * count;
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, angle);
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                    if (count < number_of_loop_rotation)
                    {
                        ++ count;
                    }

                    global_path.poses.push_back(pose);
                }
            }
            else
            {
                for (int j = 0; j < number_of_loop_rotation; ++j)
                {
                    pose.header.stamp = m_node->now();
                    double angle = m_waypoint_list[i-1][2] + theta_increment * j;
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, angle);
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();

                    pose.pose.position.x = m_waypoint_list[i-1][0] + x_increment * count;
                    pose.pose.position.y = m_waypoint_list[i-1][1] + y_increment * count;
                    if (count < number_of_loop_translation)
                    {
                        ++ count;
                    }

                    global_path.poses.push_back(pose);
                }
            }
        }
    }
    else
    {
        //send just the waypoints
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = m_global_frame;
        pose.pose.position.z = 0.0;
        global_path.header.stamp = m_node->now();
        for (size_t i = 0; i < m_waypoint_list.size() - 1; i++)
        {
            pose.pose.position.x = m_waypoint_list[i][0];
            pose.pose.position.y = m_waypoint_list[i][1];

            double angle = m_waypoint_list[i][2];
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, angle);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            global_path.poses.push_back(pose);
        }
    }
    
    //append final pose
    geometry_msgs::msg::PoseStamped final_pose;
    final_pose.header.stamp = m_node->now();
    final_pose.header.frame_id = m_global_frame;

    final_pose.pose.position.x = m_waypoint_list.back()[0];
    final_pose.pose.position.y = m_waypoint_list.back()[1];
    final_pose.pose.position.z = 0.0;

    double final_angle = m_waypoint_list.back()[2];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, final_angle);
    final_pose.pose.orientation.x = q.x();
    final_pose.pose.orientation.y = q.y();
    final_pose.pose.orientation.z = q.z();
    final_pose.pose.orientation.w = q.w();
    global_path.poses.push_back(final_pose);

    return global_path;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fixed_planner::WaypointInterpolatedPlanner, nav2_core::GlobalPlanner)