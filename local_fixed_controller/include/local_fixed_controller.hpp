#ifndef LOCAL_FIXED_CONTROLLER_HPP__
#define LOCAL_FIXED_CONTROLLER_HPP__

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace local_fixed_controller{

class LocalFixedController : public nav2_core::Controller
{
private:
    std::string m_name;
    std::shared_ptr<tf2_ros::Buffer> m_tf;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> m_costmap;
    nav2_util::LifecycleNode::SharedPtr m_node;
    std::string m_global_frame;
    nav_msgs::msg::Path m_global_path;
    int m_start_index;
    //publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_publisher;

    //Parameters
    double m_tf_tol;
    double m_radial_tol;
    int m_poses_tol;
    std::string m_topic_name;

    //Transform the path into the local frame
    nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path & path);

    nav_msgs::msg::Path planTracking(const nav_msgs::msg::Path & path);
public:
    LocalFixedController() = default;
    ~LocalFixedController() = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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

    void setPlan(const nav_msgs::msg::Path & path) override;
    
    /**
    * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
    *
    * It is presumed that the global plan is already set.
    *
    * This is mostly a wrapper for the protected computeVelocityCommands
    * function which has additional debugging info.
    *
    * @param pose Current robot pose
    * @param velocity Current robot velocity
    * @param goal_checker Pointer to the current goal checker the task is utilizing
    * @return The best command for the robot to drive
    */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    /**
    * @brief Limits the maximum linear speed of the robot.
    * @param speed_limit expressed in absolute value (in m/s)
    * or in percentage from maximum robot speed.
    * @param percentage Setting speed limit in percentage if true
    * or in absolute values in false case.
    */
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

};


}

#endif