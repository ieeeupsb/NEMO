#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class MovingRobot : public rclcpp::Node {
  private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

  public:
	MovingRobot() : Node("nemo") {
		// Initialize publisher
		publisher = this->create_publisher<geometry_msgs::msg::Twist>(
			"/model/nemo/cmd_vel", 10);

		RCLCPP_INFO(this->get_logger(), "Initialized Moving Robot node");

		move();
	}

	void move() {
		auto message = geometry_msgs::msg::Twist();
		message.linear.x = 0.5;
		message.angular.z = 0.3;

		RCLCPP_INFO(this->get_logger(), "Publishing: '%f.2' and %f.2",
					message.linear.x, message.angular.z);
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovingRobot>());
	rclcpp::shutdown();

	return 0;
}