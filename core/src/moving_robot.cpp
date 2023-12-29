#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class MovingRobot : public rclcpp::Node {
  private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;

	void move() {
		auto message = geometry_msgs::msg::Twist();
		message.linear.x = 0.5;
		message.angular.z = 0.3;

		publisher->publish(message);
	
		RCLCPP_INFO(this->get_logger(), "Publishing: '%f.2' and %f.2",
					message.linear.x, message.angular.z);
	}

	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		// Process the received odometry data
		double x = msg->pose.pose.position.x;
		double y = msg->pose.pose.position.y;
		double heading = msg->pose.pose.orientation.z;

		RCLCPP_INFO(this->get_logger(),
					"Received Odometry: x=%f, y=%f, heading=%f", x, y, heading);
	}

  public:
	MovingRobot() : Node("nemo") {
		// Initialize publisher
		publisher = this->create_publisher<geometry_msgs::msg::Twist>(
			"/model/nemo/cmd_vel", 10);

		// Initialize subscriber
		subscription = this->create_subscription<nav_msgs::msg::Odometry>(
			"/model/nemo/odometry", 10, std::bind(&MovingRobot::odomCallback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Initialized Moving Robot node");

		move();
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovingRobot>());
	rclcpp::shutdown();

	return 0;
}