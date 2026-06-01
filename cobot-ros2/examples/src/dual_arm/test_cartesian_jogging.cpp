#include <chrono>
#include <memory>
#include <thread>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DualArmCartesianJogging : public rclcpp::Node
{
public:
    DualArmCartesianJogging()
        : Node("dual_arm_cartesian_jogging")
    {
        pub_arm1_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/arm_1_cartesian_jogging_controller/cartesian_jogging/command", 10);

        pub_arm2_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/arm_2_cartesian_jogging_controller/cartesian_jogging/command", 10);

        timer_ = this->create_wall_timer(
            30ms, std::bind(&DualArmCartesianJogging::updateLoop, this));

        keyboard_thread_ = std::thread(&DualArmCartesianJogging::keyboardLoop, this);

        RCLCPP_INFO(this->get_logger(),
            "Dual Arm Cartesian Jogging Ready\n"
            "Linear:  w/s (X), a/d (Y), r/f (Z)\n"
            "Angular: i/k (Rx), j/l (Ry), u/o (Rz)\n"
            "Press x to exit");
    }

    ~DualArmCartesianJogging()
    {
        running_ = false;
        if (keyboard_thread_.joinable())
            keyboard_thread_.join();
    }

private:

    void updateLoop()
    {
        if (!publishersReady())
            return;
        
        auto now = std::chrono::steady_clock::now();
        double timeout = 0.1;

        geometry_msgs::msg::Twist msg_arm1;
        geometry_msgs::msg::Twist msg_arm2;

        if (active_axis_ != -1 &&
            std::chrono::duration<double>(now - last_key_time_).count() < timeout)
        {
            double val = direction_ * 1.0;

            // -------- LINEAR --------
            if (active_axis_ == 0) { // X
                msg_arm1.linear.x = val ;
                msg_arm2.linear.x =  val ;
            }
            if (active_axis_ == 1) { // Y
                msg_arm1.linear.y =  val ;
                msg_arm2.linear.y =  val ;
            }
            if (active_axis_ == 2) { // Z
                msg_arm1.linear.z =  val ;
                msg_arm2.linear.z =  val ;
            }

            // -------- ANGULAR --------
            if (active_axis_ == 3) { // Rx
                msg_arm1.angular.x = val;
                msg_arm2.angular.x =  val;
            }
            if (active_axis_ == 4) { // Ry
                msg_arm1.angular.y = val;
                msg_arm2.angular.y =  val;
            }
            if (active_axis_ == 5) { // Rz
                msg_arm1.angular.z = val;
                msg_arm2.angular.z =  val;
            }
        }
        else
        {
            active_axis_ = -1; // stop
        }

        pub_arm1_->publish(msg_arm1);
        pub_arm2_->publish(msg_arm2);
    }

    bool publishersReady()
    {
        if (pub_arm1_->get_subscription_count() == 0 ||
            pub_arm2_->get_subscription_count() == 0)
            {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for both cartesian controllers...");
            return false;
        }
        return true;
    }

    void keyboardLoop()
    {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        fd_set readfds;
        struct timeval tv;

        while (running_)
        {
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);

            tv.tv_sec = 0;
            tv.tv_usec = 10000;

            int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

            if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds))
            {
                char c = getchar();

                if (c == 'x')
                {
                    rclcpp::shutdown();
                    break;
                }

                handleKey(c);
                last_key_time_ = std::chrono::steady_clock::now();
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    void handleKey(char c)
    {
        // Linear
        if (c == 'w') { active_axis_ = 0; direction_ = 1; return; }
        if (c == 's') { active_axis_ = 0; direction_ = -1; return; }

        if (c == 'a') { active_axis_ = 1; direction_ = 1; return; }
        if (c == 'd') { active_axis_ = 1; direction_ = -1; return; }

        if (c == 'r') { active_axis_ = 2; direction_ = 1; return; }
        if (c == 'f') { active_axis_ = 2; direction_ = -1; return; }

        // Angular
        if (c == 'i') { active_axis_ = 3; direction_ = 1; return; }
        if (c == 'k') { active_axis_ = 3; direction_ = -1; return; }

        if (c == 'j') { active_axis_ = 4; direction_ = 1; return; }
        if (c == 'l') { active_axis_ = 4; direction_ = -1; return; }

        if (c == 'u') { active_axis_ = 5; direction_ = 1; return; }
        if (c == 'o') { active_axis_ = 5; direction_ = -1; return; }

        active_axis_ = -1;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm2_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::thread keyboard_thread_;
    bool running_ = true;

    std::chrono::steady_clock::time_point last_key_time_;
    int active_axis_ = -1;
    double direction_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualArmCartesianJogging>());
    rclcpp::shutdown();
    return 0;
}