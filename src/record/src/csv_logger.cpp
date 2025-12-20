#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "ros2_interface/msg/robot.hpp"
#include "ros2_interface/msg/personpos.hpp"

#include <fstream>
#include <iomanip>

class CsvLogger : public rclcpp::Node
{
public:
    CsvLogger() : Node("csv_logger")
    {
        file_.open("bag_output/data.csv", std::ios::out);
        file_ << std::fixed << std::setprecision(6);

        // CSV header
        file_ <<
            "time,topic,"
            "robot_pose_x,robot_pose_y,robot_pose_theta,"
            "robot_vel_linear,robot_vel_angular,"
            "robot_mode,robot_battery,robot_charge,robot_emergency,"
            "person_x,person_y,person_theta,person_radius\n";

        sub_robot_ = this->create_subscription<ros2_interface::msg::Robot>(
            "/reeman/robot_info", 10,
            std::bind(&CsvLogger::robotCallback, this, std::placeholders::_1));

        sub_person_ = this->create_subscription<ros2_interface::msg::Personpos>(
            "/uwb/person_pos", 10,
            std::bind(&CsvLogger::personCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CSV Logger started");
    }

    ~CsvLogger()
    {
        file_.close();
    }

private:
    std::ofstream file_;

    rclcpp::Subscription<ros2_interface::msg::Robot>::SharedPtr sub_robot_;
    rclcpp::Subscription<ros2_interface::msg::Personpos>::SharedPtr sub_person_;

    // ===== Robot callback =====
    void robotCallback(const ros2_interface::msg::Robot::SharedPtr msg)
    {
        double t = this->now().seconds();

        file_ << t << ",robot,"
              << msg->pose_x << ","
              << msg->pose_y << ","
              << msg->pose_theta << ","
              << msg->vel_linear << ","
              << msg->vel_angular << ","
              << static_cast<int>(msg->mode) << ","
              << static_cast<int>(msg->battery_level) << ","
              << static_cast<int>(msg->charge_flag) << ","
              << static_cast<int>(msg->emergency_flag) << ","
              << ",,,\n";
    }

    // ===== PersonPos callback =====
    void personCallback(const ros2_interface::msg::Personpos::SharedPtr msg)
    {
        double t = this->now().seconds();

        file_ << t << ",person,"
              << ",,,,,,,,"
              << msg->uwb_pose.x << ","
              << msg->uwb_pose.y << ","
              << msg->uwb_pose.theta << ","
              << msg->radius << "\n";
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CsvLogger>());
    rclcpp::shutdown();
    return 0;
}

