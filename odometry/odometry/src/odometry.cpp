#include <odometry/odometry.hpp>
Odometry::Odometry(const rclcpp::NodeOptions &options) : Node("odometry_node", options),
                                                         sync(ApproximateSyncPolicy(2), abs_encoder1_sub, abs_encoder2_sub),
                                                         velocity_rolling_window_size_(ROLLING_WINDOW_SIZE),
                                                         linear_accumulator_(ROLLING_WINDOW_SIZE),
                                                         angular_accumulator_(ROLLING_WINDOW_SIZE)
{
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/adir/odom", 10);
    vehicle_speed_pub = this->create_publisher<geometry_msgs::msg::Twist>("/vehicle_speed", 10);
    can_pub = this->create_publisher<adir_ros_can_interfaces::msg::Frame>("/interface/can/send", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    serviceResetOdometry = this->create_service<odometry_interfaces::srv::ResetOdometry>(
        "ResetOdometry",
        std::bind(&Odometry::resetOdometry, this, std::placeholders::_1, std::placeholders::_2));

    abs_encoder1_sub.subscribe(this, "motor_controller/motor1/abs_encoder");
    abs_encoder2_sub.subscribe(this, "motor_controller/motor2/abs_encoder");
    sync.registerCallback(&Odometry::encoderCallback, this);

    // get parameters
    this->declare_parameter("encoder_ppr", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("wheel_circumference", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("compensation_factor", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("gear_ratio", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);

    encoderPpr = (this->get_parameter("encoder_ppr").as_int());
    wheelCircum = (this->get_parameter("wheel_circumference").as_double());
    compensationFactor = (this->get_parameter("compensation_factor").as_double());
    gearRatio = (this->get_parameter("gear_ratio").as_double());
    trackWidth = (this->get_parameter("track_width").as_double());

    encoderCpr = quadrature_mode * encoderPpr;
    ticksPerMeter = gearRatio * encoderCpr / wheelCircum;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    init();

    timer_ = this->create_wall_timer(
        100ms, std::bind(&Odometry::publishOdomAndTf, this));
}

bool Odometry::resetOdometry(const std::shared_ptr<odometry_interfaces::srv::ResetOdometry::Request> req, const std::shared_ptr<odometry_interfaces::srv::ResetOdometry::Response> res)
{
    init();
    res->status = true;
    return true;
}

void Odometry::init()
{
    resetAccumulators();

    prev_abs_encoder_1 = 0;
    prev_abs_encoder_2 = 0;

    vx = 0.0;
    vyaw = 0.0;

    robot_last_x = 0.0;
    robot_last_y = 0.0;
    robot_last_theta = 0.0;

    robot_x = 0.0;
    robot_y = 0.0;
    robot_theta = 0.0;

    then = this->get_clock()->now();
    t_next = this->get_clock()->now() + rclcpp::Duration::from_seconds(1.0/rate);
}

void Odometry::resetAccumulators()
{
    linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

void Odometry::encoderCallback(const StampedInt32::ConstSharedPtr &abs_encoder1, const StampedInt32::ConstSharedPtr &abs_encoder2)
{
    rclcpp::Time now = this->get_clock()->now();

    if (now < t_next)
        return;

    try
    {
        dT = now.seconds() - then.seconds();
       
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }

    /* -------------------------------------------------------------------------- */
    /*                              Start Calculation                             */
    /* -------------------------------------------------------------------------- */

    double dL = (abs_encoder1->data - prev_abs_encoder_1) / ticksPerMeter;
    double dR = (abs_encoder2->data - prev_abs_encoder_2) / ticksPerMeter;
    prev_abs_encoder_1 = abs_encoder1->data;
    prev_abs_encoder_2 = abs_encoder2->data;

    double translation = 0.5 * (dR + dL);
    double rotation = (dL - dR) / trackWidth;

    if (translation != 0)
    {
        if (fabs(rotation) < 1e-6) 
        {
            robot_x += cos(robot_theta) * translation;
            robot_y += sin(robot_theta) * translation;
        }
        else
        {
            robot_last_theta = robot_theta;
            const double R = translation / rotation;
            robot_theta = NORMALIZE(robot_theta + rotation); // rad
            robot_x += R * (sin(robot_theta) - sin(robot_last_theta));
            robot_y += -R * (cos(robot_theta) - cos(robot_last_theta));
        }
    }

    // Calculate velocities
    
    linear_accumulator_.accumulate(translation / dT); 
    angular_accumulator_.accumulate((robot_theta - robot_last_theta) / dT);

    vx = linear_accumulator_.getRollingMean();
    vyaw = angular_accumulator_.getRollingMean();

    robot_last_x = robot_x;
    /* -------------------------------------------------------------------------- */
    /*                               End Calculation                              */
    /* -------------------------------------------------------------------------- */

    then = now;
}

void Odometry::publishOdomAndTf()
{
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::Quaternion odom_quat;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_theta); // rad

    odom_quat.x = q.x();
    odom_quat.y = q.y();
    odom_quat.z = q.z();
    odom_quat.w = q.w();

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = robot_x;
    odom_trans.transform.translation.y = robot_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    tf_broadcaster->sendTransform(odom_trans);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = robot_x;
    odom.pose.pose.position.y = robot_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vyaw;

    odom_pub->publish(odom);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx;
    twist.angular.z = vyaw;
    vehicle_speed_pub->publish(twist);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Odometry)
