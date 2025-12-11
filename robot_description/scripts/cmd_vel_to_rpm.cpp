#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class CmdVelToRPM : public rclcpp::Node
{
public:
    CmdVelToRPM() : Node("cmd_vel_to_rpm")
    {
        r_ = declare_parameter<double>("wheel_radius", 0.1);
        L_ = declare_parameter<double>("wheel_separation", 0.45);

        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToRPM::callback, this, std::placeholders::_1));

        pub_l_ = create_publisher<std_msgs::msg::Float64>("/left_motor_rpm", 10);
        pub_r_ = create_publisher<std_msgs::msg::Float64>("/right_motor_rpm", 10);
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double w = msg->angular.z;

        double wl = (v - w * L_ / 2.0) / r_;
        double wr = (v + w * L_ / 2.0) / r_;

        double rpm_l = wl * 60.0 / (2.0 * M_PI);
        double rpm_r = wr * 60.0 / (2.0 * M_PI);

        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;

        left_msg.data = rpm_l;
        right_msg.data = rpm_r;

        pub_l_->publish(left_msg);
        pub_r_->publish(right_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_r_;

    double r_, L_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToRPM>());
    rclcpp::shutdown();
    return 0;
}
