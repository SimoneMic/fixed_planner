#include <vector>

#include "local_fixed_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>

namespace local_fixed_controller{

void LocalFixedController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    m_node = parent.lock();
    m_name = name;
    m_costmap = costmap_ros;
    m_global_frame = costmap_ros->getGlobalFrameID();
    m_tf = tf;

    //Parameters
    nav2_util::declare_parameter_if_not_declared(
        m_node, m_name + ".topic_name", rclcpp::ParameterValue("/tracked_plan"));
    m_node->get_parameter(m_name + ".topic_name", m_topic_name);
    nav2_util::declare_parameter_if_not_declared(
        m_node, m_name + ".number_goal_poses_tolerance",
        rclcpp::ParameterValue(2));
    m_node->get_parameter(m_name + ".number_goal_poses_tolerance", m_poses_tol);
    nav2_util::declare_parameter_if_not_declared(
        m_node, m_name + ".radial_tolerance", rclcpp::ParameterValue(0.1));
    m_node->get_parameter(m_name + ".radial_tolerance", m_radial_tol);
    nav2_util::declare_parameter_if_not_declared(
        m_node, m_name + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    m_node->get_parameter(m_name + ".transform_tolerance", m_tf_tol);

    m_publisher = m_node->create_publisher<nav_msgs::msg::Path>(m_topic_name, 10);
}



void LocalFixedController::setPlan(const nav_msgs::msg::Path & path)
{
    if (path.poses.empty()) 
        throw nav2_core::PlannerException("Received plan with zero length");
  
    // Transform global path into the robot's frame
    m_global_path = LocalFixedController::transformGlobalPlan(path);
}

nav_msgs::msg::Path LocalFixedController::transformGlobalPlan(const nav_msgs::msg::Path & path)
{
    nav_msgs::msg::Path out_path;
    //Transform the path in local frame
    out_path.header.frame_id = m_costmap->getBaseFrameID();
    out_path.header.stamp = path.header.stamp;
    out_path.poses.resize(path.poses.size());
    std::vector<geometry_msgs::msg::PoseStamped>::iterator out_iter = out_path.poses.begin();
    for (std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it = path.poses.begin();
         it != path.poses.end();
         ++it, ++out_iter)
    {
        tf2::doTransform(it->pose, 
                        out_iter->pose, 
                        m_tf->lookupTransform(out_path.header.frame_id, path.header.frame_id, tf2::getTimestamp(it), 
                        tf2::durationFromSec(m_tf_tol)));
    }
    //remove the already passed poses
    out_path = planTracking(out_path);

    //publish transformed and tracked path
    m_publisher->publish(out_path);

    return out_path;
}

nav_msgs::msg::Path LocalFixedController::planTracking(const nav_msgs::msg::Path & path)
{
    nav_msgs::msg::Path out_path;
    out_path.header.frame_id = path.header.frame_id;
    out_path.header.stamp = path.header.stamp;

    //find the closest pose on the path to the robot frame
    double min_dist = 1e10;
    int min_index = 0;
    for (int i = m_start_index; i < path.poses.size(); ++i)
    {
        double distance = std::sqrt(std::pow(path.poses[i].pose.position.x, 2) + std::pow(path.poses[i].pose.position.y, 2));
        if (distance < min_dist)
        {
            min_dist = distance;
            min_index = i;
        }
    }
    //verify that the closest pose is one of the next ones in the ordered path
    if (min_index > m_start_index)
    {
        m_start_index = min_index;
    }

    //extract only the poses following the closest pose to the robot frame
    out_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(&path.poses[min_index], &path.poses.back());

    //check if we have reached the end of the path
    if (m_start_index > path.poses.size() - m_poses_tol)
    {
        //reset starting index
        m_start_index = 0;
    }
    
    return out_path;
}

geometry_msgs::msg::TwistStamped LocalFixedController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    geometry_msgs::msg::TwistStamped msg_out;
    msg_out.header.frame_id = m_costmap->getBaseFrameID();
    msg_out.header.stamp = m_node->now();
    double d_x = m_global_path.poses[1].pose.position.x - pose.pose.position.x;
    double d_y = m_global_path.poses[1].pose.position.y - pose.pose.position.y;
    msg_out.twist.linear.x = d_x/10;
    msg_out.twist.linear.y = d_y/10;
    msg_out.twist.linear.z = .0;
    msg_out.twist.angular.x = .0;
    msg_out.twist.angular.y = .0;
    msg_out.twist.angular.z = .0;
    return msg_out;
}

void LocalFixedController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(local_fixed_controller::LocalFixedController, nav2_core::Controller)