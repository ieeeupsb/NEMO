#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <memory>

// OpenCV
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

struct Position {
	double x, y;
};

Position MARKER_POSITIONS[] = {
	{-695, 355},  {-545, 355},	{-395, 355},  {-245, 355},	{0, 355},	  {695, 355},	{0, 150},
	{227, 150},	  {468, 150},	{695, 150},	  {-695, 0},	{-468, 0},	  {-227, 0},	{0, 0},
	{227, 0},	  {468, 0},		{695, 0},	  {-695, -150}, {-468, -150}, {-227, -150}, {0, -150},
	{-695, -355}, {0, -355},	{245, -355},  {395, -355},	{545, -355},  {695, -355},	{227, -355},
	{468, -355},  {-468, -355}, {-227, -355}, {-695, 150},	{695, -150},
};

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
		RCLCPP_INFO(this->get_logger(), "Camera callback");

		// Initialize the ArUco dictionary
		cv::aruco::Dictionary aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
		cv::Ptr<cv::aruco::Dictionary> ptr_aruco_dict = cv::makePtr<cv::aruco::Dictionary>(aruco_dict);

		// Convert the ROS image message to an OpenCV image
		cv_bridge::CvImagePtr cv_ptr;

		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}

		catch (cv_bridge::Exception &e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());

			return;
		}

		// Detect the markers in the image
		std::vector<int> marker_ids;
		std::vector<std::vector<cv::Point2f>> marker_corners;

		cv::aruco::detectMarkers(cv_ptr->image, ptr_aruco_dict, marker_corners, marker_ids);

		// Print the detected markers
		for (size_t i = 0; i < marker_ids.size(); i++) {
			RCLCPP_INFO(this->get_logger(), "Detected marker: %d", marker_ids[i]);
		}
	}

	void moveToPosition(double x, double y) {
		// Calculate the delta to target
		double delta_x = x - location.x;
		double delta_y = y - location.y;

		// Calculate the angle and distance to target
		double target_angle = atan2(delta_y, delta_x);
		// double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

		// Move robot
		move(1.0, target_angle);

		// Update location
		location.x = x;
		location.y = y;

		RCLCPP_INFO(this->get_logger(), "Moving to tile: x=%f, y=%f", x, y);
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
		// odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
		// 	"/model/nemo/odometry", 10, std::bind(&NemoController::odomCallback, this, std::placeholders::_1));

		camera_subscription = image_transport.subscribe(
			"/camera", 1, std::bind(&NemoController::cameraCallback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Initialized NEMO controller node");
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<NemoController>());
	rclcpp::shutdown();

	return 0;
}