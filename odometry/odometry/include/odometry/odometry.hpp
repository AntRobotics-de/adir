#ifndef ODOMETRY_ODOMETRY_HPP
#define ODOMETRY_ODOMETRY_HPP

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))
#define ROLLING_WINDOW_SIZE 20

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ros_can_interfaces/msg/frame.hpp>
#include <can_commands_interfaces/msg/stamped_int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <odometry_interfaces/srv/reset_odometry.hpp>
#include <rcppmath/rolling_mean_accumulator.hpp>
#include <boost/function.hpp>
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
    Odometry(const rclcpp::NodeOptions&);
    void init();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    double compensationFactor = -3.5;
    double trackWidth;
    double wheelCircum;
    double gearRatio;
    int    encoderPpr = 1024;
    int    quadrature_mode = 4;
    int    encoderCpr;
    double ticksPerMeter;

    double prev_abs_encoder_1;
    double prev_abs_encoder_2;

    double robot_x;
    double robot_y;
    double robot_theta;

    double robot_last_x;
    double robot_last_y;
    double robot_last_theta;
    double vx;
    double vyaw;

    double rate = 20;
    rclcpp::Time t_next;
    rclcpp::Time then;
    double dT;

    using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
    size_t velocity_rolling_window_size_;
    RollingMeanAccumulator linear_accumulator_;
    RollingMeanAccumulator angular_accumulator_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vehicle_speed_pub;
    rclcpp::Publisher<ros_can_interfaces::msg::Frame>::SharedPtr can_pub;
    rclcpp::Service<odometry_interfaces::srv::ResetOdometry>::SharedPtr serviceResetOdometry;

    typedef can_commands_interfaces::msg::StampedInt32 StampedInt32;
    message_filters::Subscriber<StampedInt32> abs_encoder1_sub;
    message_filters::Subscriber<StampedInt32> abs_encoder2_sub;

    typedef message_filters::sync_policies::ApproximateTime<StampedInt32, StampedInt32> ApproximateSyncPolicy;
    message_filters::Synchronizer<ApproximateSyncPolicy> sync;

    bool resetOdometry(const std::shared_ptr<odometry_interfaces::srv::ResetOdometry::Request> req, const std::shared_ptr<odometry_interfaces::srv::ResetOdometry::Response> res);
    void publishOdomAndTf();
    void encoderCallback(const StampedInt32::ConstSharedPtr &abs_encoder1, const StampedInt32::ConstSharedPtr &abs_encoder2);
    void resetAccumulators();
};

#endif
