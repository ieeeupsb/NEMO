#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <image_transport/image_transport.h>
#include <memory>

struct Position {
	double x, y;
};

Position TILE_MAP[] = {{1.0, 0.5},	{2.5, 0.5},	 {4.0, 0.5},  {5.5, 0.5},  {8.0, 0.5},	{10.3, 0.5}, {12.7, 0.5},
					   {15.0, 0.5}, {1.0, 2.5},	 {8.0, 2.5},  {10.3, 2.5}, {12.7, 2.5}, {15.0, 2.5}, {1.0, 4.0},
					   {3.3, 4.0},	{5.7, 4.0},	 {8.0, 4.0},  {10.3, 4.0}, {12.7, 4.0}, {15.0, 4.0}, {1.0, 5.5},
					   {3.3, 5.5},	{5.7, 5.5},	 {8.0, 5.5},  {15, 5.5},   {1.0, 7.5},	{3.3, 7.5},	 {5.7, 7.5},
					   {8.0, 7.5},	{10.5, 7.5}, {12.0, 7.5}, {13.5, 7.5}, {15.0, 7.5}};

class NemoController : public rclcpp::Node {
  private:
	rclcpp::Node::SharedPtr node_handle;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;

	// Image transport
	image_transport::ImageTransport image_transport;
	image_transport::Subscriber camera_subscription;

	Position location;
	double rotation;

	void move(double linear_velocity, double angular_velocity) {
		auto message = geometry_msgs::msg::Twist();
		message.linear.x = linear_velocity;
		message.angular.z = angular_velocity;

		publisher->publish(message);

		RCLCPP_INFO(this->get_logger(), "Publishing: '%f.2' and %f.2", message.linear.x, message.angular.z);
	}

	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		// Process the received odometry data
		double x = msg->pose.pose.position.x;
		double y = msg->pose.pose.position.y;
		double heading = msg->pose.pose.orientation.z;

		location.x += x;
		location.y += y;
		rotation += heading;

		RCLCPP_INFO(this->get_logger(), "Received Odometry: x=%f, y=%f, heading=%f", x, y, heading);
	}

	void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
		// TODO:
	}

	void moveToPosition(double x, double y) {
		// Calculate the delta to target
		double delta_x = x - location.x;
		double delta_y = y - location.y;

		// Calculate the angle and distance to target
		double target_angle = atan2(delta_y, delta_x);
		double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

		// Move robot
		move(1.0, target_angle);

		// Update location
		location.x = x;
		location.y = y;

		RCLCPP_INFO(this->get_logger(), "Moving to tile: x=%d, y=%d", x, y);
	}

	void toggleMagnet() {
		// TODO:
	}

  public:
	NemoController()
		: Node("nemo"), node_handle(std::shared_ptr<NemoController>(this, [](auto *) {})),
		  image_transport(node_handle) {
		// Set initial location
		location = {4, 4};

		// Set initial rotation
		rotation = 0;

		// Initialize publishers
		publisher = this->create_publisher<geometry_msgs::msg::Twist>("/model/nemo/cmd_vel", 10);

		// Initialize subscribers
		odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
			"/model/nemo/odometry", 10, std::bind(&NemoController::odomCallback, this, std::placeholders::_1));

		camera_subscription = image_transport.subscribe(
			"/model/nemo/camera", 1, std::bind(&NemoController::cameraCallback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Initialized NEMO controller node");

		moveToPosition(12, 6);
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<NemoController>());
	rclcpp::shutdown();

	return 0;
}