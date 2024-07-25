#pragma once

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "antrobotics_can_commands_interfaces/msg/stamped_int32.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_ros/transform_broadcaster.h"
#include "diff_drive_controller/odometry.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class OdometryROS : public rclcpp::Node
{
public:
    OdometryROS(const rclcpp::NodeOptions &);

private:
    diff_drive_controller::Odometry odometry_;

    typedef antrobotics_can_commands_interfaces::msg::StampedInt32 StampedInt32;
    std::shared_ptr<message_filters::Subscriber<StampedInt32>> absolute_encoder1_subscriber_;
    std::shared_ptr<message_filters::Subscriber<StampedInt32>> absolute_encoder2_subscriber_;
    typedef message_filters::sync_policies::ApproximateTime<StampedInt32, StampedInt32> ApproximateSyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> reset_encoder_ticks_publisher_ = nullptr;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odometry_service_;

    bool resetOdometryServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void publishOdom(const rclcpp::Time &);
    void publishOdomTf(const rclcpp::Time &);
    void encoderCallback(const StampedInt32::ConstSharedPtr &abs_encoder1, const StampedInt32::ConstSharedPtr &abs_encoder2);
    void resetEncoderTicks();
    void resetOdometry();
    void declareAndGetParameters();

    tf2::Quaternion orientation_;

    rclcpp::Time previous_update_time_{this->now()};
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);

    double track_width_;
    double wheel_radius_;
    double wheel_separation_;
    double wheel_circumference_;
    double gear_ratio_;
    int velocity_rolling_window_size_;
    int encoder_ppr_;
    int quadrature_mode_ = 4;
    int encoder_cpr_;
    int wheel_per_side_;
    double publish_rate_;
    double ticks_per_meter_;
    double ticks_per_rev_;

    bool publish_odom_tf_ = true;
    std::string odom_frame_id_;
    std::string child_frame_id_;
    std::string reset_encoder_ticks_topic_;
    std::string absolute_encoder_1_topic_;
    std::string absolute_encoder_2_topic_;
};