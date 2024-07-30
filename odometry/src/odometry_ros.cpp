#include <odometry/odometry_ros.hpp>

using namespace std::placeholders;

OdometryROS::OdometryROS(const rclcpp::NodeOptions &options) : Node("odometry_node", options)
{
    declareAndGetParameters();
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

    wheel_circumference_ = 2 * M_PI * wheel_radius_;
    encoder_cpr_ = quadrature_mode_ * encoder_ppr_;
    ticks_per_meter_ = gear_ratio_ * encoder_cpr_ / wheel_circumference_;
    ticks_per_rev_ = gear_ratio_ * encoder_cpr_;

    // initialize odometry object
    odometry_.setWheelParams(wheel_separation_, wheel_radius_, wheel_radius_);
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size_);

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 2);
    realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);
    auto &odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = odom_frame_id_;
    odometry_message.child_frame_id = child_frame_id_;

    odometry_transform_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 2);
    realtime_odometry_transform_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);
    // keeping track of odom and base_link transforms only
    auto &odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_frame_id_;
    odometry_transform_message.transforms.front().child_frame_id = child_frame_id_;

    reset_odometry_service_ = this->create_service<std_srvs::srv::Trigger>("ResetOdometry", std::bind(&OdometryROS::resetOdometryServiceCallback, this, _1, _2));

    reset_encoder_ticks_publisher_ = this->create_publisher<std_msgs::msg::Bool>(reset_encoder_ticks_topic_, 10);
    
    absolute_encoder1_subscriber_ = std::make_shared<message_filters::Subscriber<StampedInt32>>(this, absolute_encoder_1_topic_);
    absolute_encoder2_subscriber_ = std::make_shared<message_filters::Subscriber<StampedInt32>>(this, absolute_encoder_2_topic_);
    sync = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(2), *absolute_encoder1_subscriber_, *absolute_encoder2_subscriber_);
    sync->registerCallback(&OdometryROS::encoderCallback, this);

    resetOdometry();
}

void OdometryROS::resetOdometry()
{
    odometry_.resetOdometry();
    resetEncoderTicks();
}

bool OdometryROS::resetOdometryServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    resetOdometry();

    res->success = true;
    res->message = "Odometry has been reset.";

    return true;
}

void OdometryROS::declareAndGetParameters()
{
    // get parameters
    this->declare_parameter("encoder_ppr", 1024);
    this->declare_parameter("velocity_rolling_window_size", 10);
    this->declare_parameter("wheel_per_side", 2);
    this->declare_parameter("gear_ratio", 58.0);
    this->declare_parameter("wheel_radius", 0.174 - 0.002);
    this->declare_parameter("wheel_separation", 0.6 + 0.88);
    this->declare_parameter("publish_rate", 30.0);
    this->declare_parameter("child_frame_id", "base_link");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("absolute_encoder_1_topic", "roboteq/motor1/abs_encoder");
    this->declare_parameter("absolute_encoder_2_topic", "roboteq/motor2/abs_encoder");
    this->declare_parameter("reset_encoder_ticks_topic", "roboteq/encoder/reset");
    this->declare_parameter("publish_odom_tf", true);

    encoder_ppr_ = (this->get_parameter("encoder_ppr").as_int());
    velocity_rolling_window_size_ = (this->get_parameter("velocity_rolling_window_size").as_int());
    wheel_per_side_ = (this->get_parameter("wheel_per_side").as_int());
    gear_ratio_ = (this->get_parameter("gear_ratio").as_double());
    wheel_radius_ = (this->get_parameter("wheel_radius").as_double());
    wheel_separation_ = (this->get_parameter("wheel_separation").as_double());
    publish_rate_ = (this->get_parameter("publish_rate").as_double());
    odom_frame_id_ = (this->get_parameter("odom_frame_id").as_string());
    child_frame_id_ = (this->get_parameter("child_frame_id").as_string());
    absolute_encoder_1_topic_ = (this->get_parameter("absolute_encoder_1_topic").as_string());
    absolute_encoder_2_topic_ = (this->get_parameter("absolute_encoder_2_topic").as_string());
    reset_encoder_ticks_topic_ = (this->get_parameter("reset_encoder_ticks_topic").as_string());
    publish_odom_tf_ = (this->get_parameter("publish_odom_tf").as_bool());
}

void OdometryROS::resetEncoderTicks()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    reset_encoder_ticks_publisher_->publish(msg);
}

void OdometryROS::encoderCallback(const StampedInt32::ConstSharedPtr &abs_encoder1, const StampedInt32::ConstSharedPtr &abs_encoder2)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Received encoder data: " << abs_encoder1->data << "," << abs_encoder2->data);
    auto const current_time = this->now();

    double left_pos = 0.0;
    double right_pos = 0.0;
    left_pos = (abs_encoder1->data) / ticks_per_rev_;
    right_pos = (abs_encoder2->data) / ticks_per_rev_;
    double left_pos_deg = left_pos * 360;
    double right_pos_deg = right_pos * 360;
    double left_pos_rad = left_pos_deg * M_PI / 180;
    double right_pos_rad = right_pos_deg * M_PI / 180;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Left pos rad: " << left_pos_rad << ", Right pos rad: " << right_pos_rad);

    odometry_.update(left_pos_rad, right_pos_rad, current_time);

    orientation_.setRPY(0.0, 0.0, odometry_.getHeading());

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Odometry: x=" << odometry_.getX() << ", y=" << odometry_.getY() << ", heading=" << odometry_.getHeading());

    bool should_publish = false;
    try
    {
        if (previous_update_time_ + publish_period_ < current_time)
        {
            previous_update_time_ += publish_period_;
            should_publish = true;
        }
    }
    catch (const std::runtime_error &)
    {
        // Handle exceptions when the time source changes and initialize publish timestamp
        previous_update_time_ = current_time;
        should_publish = true;
    }

    if (should_publish)
    {
        publishOdom(current_time);
        if (publish_odom_tf_)
        {
            publishOdomTf(current_time);
        }
    }
}

void OdometryROS::publishOdom(const rclcpp::Time &time)
{
    if (realtime_odometry_publisher_->trylock())
    {
        auto &odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.stamp = time;
        odometry_message.pose.pose.position.x = odometry_.getX();
        odometry_message.pose.pose.position.y = odometry_.getY();
        odometry_message.pose.pose.orientation.x = orientation_.x();
        odometry_message.pose.pose.orientation.y = orientation_.y();
        odometry_message.pose.pose.orientation.z = orientation_.z();
        odometry_message.pose.pose.orientation.w = orientation_.w();
        odometry_message.twist.twist.linear.x = odometry_.getLinear();
        odometry_message.twist.twist.angular.z = odometry_.getAngular();
        realtime_odometry_publisher_->unlockAndPublish();
    }
}

void OdometryROS::publishOdomTf(const rclcpp::Time &time)
{
    if (publish_odom_tf_ && realtime_odometry_transform_publisher_->trylock())
    {
        auto &transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odometry_.getX();
        transform.transform.translation.y = odometry_.getY();
        transform.transform.rotation.x = orientation_.x();
        transform.transform.rotation.y = orientation_.y();
        transform.transform.rotation.z = orientation_.z();
        transform.transform.rotation.w = orientation_.w();
        realtime_odometry_transform_publisher_->unlockAndPublish();
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(OdometryROS)
