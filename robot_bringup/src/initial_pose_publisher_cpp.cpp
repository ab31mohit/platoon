#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PoseInitializationPublisher : public rclcpp::Node
{
public:
    PoseInitializationPublisher()
        : Node("initial_pose_publisher"), initial_pose_published_(false)
    {
        // Declare parameters for initial pose
        this->declare_parameter<double>("odometry.initial_transform.x", 0.0);
        this->declare_parameter<double>("odometry.initial_transform.y", 0.0);
        this->declare_parameter<double>("odometry.initial_transform.yaw", 0.0);

        // Get initial pose parameters
        this->get_parameter_or<double>("odometry.initial_transform.x", initial_pose_.x, 0.0);
        this->get_parameter_or<double>("odometry.initial_transform.y", initial_pose_.y, 0.0);
        this->get_parameter_or<double>("odometry.initial_transform.yaw", initial_pose_.z, 0.0); // Using z for yaw

        RCLCPP_INFO(this->get_logger(), "Loaded initial pose: x=%f, y=%f, yaw=%f",
                    initial_pose_.x, initial_pose_.y, initial_pose_.z);

        // Create publisher for pose relocalization
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("pose_relocalization", 10);

        // Subscribe to odometry to ensure the odometry node is ready
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&PoseInitializationPublisher::odom_callback, this, std::placeholders::_1));

        // Timer as a backup to ensure publishing happens
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PoseInitializationPublisher::timer_callback, this));
    }

private:
    void publish_initial_pose()
    {
        if (!initial_pose_published_)
        {
            publisher_->publish(initial_pose_);
            RCLCPP_INFO(this->get_logger(), "Published initial pose: x=%f, y=%f, yaw=%f",
                        initial_pose_.x, initial_pose_.y, initial_pose_.z);
            initial_pose_published_ = true;

            // Stop timer after publishing
            timer_->cancel();
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
    {
        if (!initial_pose_published_)
        {
            publish_initial_pose();
        }
    }

    void timer_callback()
    {
        if (!initial_pose_published_)
        {
            publish_initial_pose();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Point initial_pose_;
    bool initial_pose_published_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseInitializationPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
