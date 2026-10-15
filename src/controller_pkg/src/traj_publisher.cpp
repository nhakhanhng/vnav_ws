#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>

#define STATIC_POSE 0

#define PI M_PI

#define TFOUTPUT 1

class TrajPublisherNode : public rclcpp::Node
{
public:
    TrajPublisherNode() : Node("traj_publisher_node") {
        desired_state_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
            "desired_state", 1);
#if TFOUTPUT
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
#endif
        // A wall timer lets the executor process /clock between publications.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&TrajPublisherNode::publish_trajectory, this));
    }

private:
    void publish_trajectory() {
        const auto stamp = this->now();
        const int64_t now_ns = stamp.nanoseconds();
        if (now_ns == 0) {
            return;  // Wait for the first /clock message when use_sim_time is enabled.
        }

        if (has_last_time_) {
            const int64_t delta_ns = now_ns - last_time_ns_;
            if (delta_ns == 0) {
                return;
            }
            // Keep the path continuous across clock resets and large corrections.
            // TESSE normally advances /clock in steps of about 0.05 s.
            if (delta_ns > 0 && delta_ns <= 200000000LL) {
                phase_seconds_ += static_cast<double>(delta_ns) * 1e-9;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Clock jumped by %.3f s; holding desired position",
                    static_cast<double>(delta_ns) * 1e-9);
            }
        }
        last_time_ns_ = now_ns;
        has_last_time_ = true;

        const double t = phase_seconds_;
        geometry_msgs::msg::TransformStamped desired_pose;
        desired_pose.header.stamp = stamp;
        desired_pose.header.frame_id = "world";
        desired_pose.child_frame_id = "av-desired";

        geometry_msgs::msg::Twist velocity;
        geometry_msgs::msg::Twist acceleration;
#if STATIC_POSE
        desired_pose.transform.translation.z = 2.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, PI / 4.0);
#else
        const double radius = 5.0;
        const double time_scale = 2.0;
        desired_pose.transform.translation.x = radius * std::sin(t / time_scale);
        desired_pose.transform.translation.y = radius * std::cos(t / time_scale);
        desired_pose.transform.translation.z = 2.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, -t / time_scale);
        velocity.linear.x = radius * std::cos(t / time_scale) / time_scale;
        velocity.linear.y = -radius * std::sin(t / time_scale) / time_scale;
        velocity.angular.z = -1.0 / time_scale;
        acceleration.linear.x = -radius * std::sin(t / time_scale) / (time_scale * time_scale);
        acceleration.linear.y = -radius * std::cos(t / time_scale) / (time_scale * time_scale);
#endif
        desired_pose.transform.rotation.x = q.x();
        desired_pose.transform.rotation.y = q.y();
        desired_pose.transform.rotation.z = q.z();
        desired_pose.transform.rotation.w = q.w();

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.push_back(desired_pose.transform);
        msg.velocities.push_back(velocity);
        msg.accelerations.push_back(acceleration);
        desired_state_pub_->publish(msg);
#if TFOUTPUT
        tf_broadcaster_->sendTransform(desired_pose);
#endif
    }

    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
#if TFOUTPUT
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
#endif
    double phase_seconds_ = 0.0;
    int64_t last_time_ns_ = 0;
    bool has_last_time_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                               // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<TrajPublisherNode>());    // Spin the node so it processes callbacks
    rclcpp::shutdown();                                     // Shutdown the ROS 2 system when done
    return 0;
}