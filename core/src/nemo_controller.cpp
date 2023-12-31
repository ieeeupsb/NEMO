#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "image_transport/image_transport.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

double MARKER_LENGTH = 0.06;

class NemoController : public rclcpp::Node {
  private:
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
	rclcpp::Node::SharedPtr node_handle;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;

	// Camera
	image_transport::ImageTransport image_transport;
	image_transport::Subscriber camera_subscription;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;
	cv::Mat camera_matrix, distortion_coeffs;	// Camera parameters

	Position location;
	double rotation;

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

		// Pose estimation
		cv::Vec3d rvec, tvec;
		std::vector<cv::Vec3d> rvecs, tvecs;

		cv::Mat object_points(4, 1, CV_32FC3);

		for (size_t i = 0; i < marker_ids.size(); i++) {
			RCLCPP_INFO(this->get_logger(), "Detected marker: %d", marker_ids[i]);

			// cv::solvePnP(object_points, marker_corners.at(i), camera_matrix, distortion_coeffs, rvec, tvec, false,
			// 			 cv::SOLVEPNP_IPPE_SQUARE);

			// rvecs.push_back(rvec);
			// tvecs.push_back(tvec);

			// RCLCPP_INFO(this->get_logger(), "rvec: %f, %f, %f", rvec[0], rvec[1], rvec[2]);
			// RCLCPP_INFO(this->get_logger(), "tvec: %f, %f, %f", tvec[0], tvec[1], tvec[2]);
		}

		// TODO: Estimate world position
	}

	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo msg) {
		// Obtain camera intrinsic parameters
		camera_matrix = cv::Mat(3, 3, CV_64F, (void *) msg.k.data()).clone();
		distortion_coeffs = cv::Mat(1, 5, CV_64F, (void *) msg.d.data()).clone();
	}

	void moveToPosition(double target_x, double target_y, double target_yaw = 0.0) {
		// Set the goal coordinates and yaw
		geometry_msgs::msg::PoseStamped goal_pose;
		goal_pose.header.frame_id = "map";
		goal_pose.pose.position.x = target_x;
		goal_pose.pose.position.y = target_y;

		tf2::Quaternion quaternion;
		quaternion.setRPY(0, 0, target_yaw);
		goal_pose.pose.orientation = tf2::toMsg(quaternion);

		// Publish the goal
		goal_publisher->publish(goal_pose);

		RCLCPP_INFO(this->get_logger(), "Moving to position: x=%f, y=%f", target_x, target_y);
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
		goal_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

		// Initialize subscribers
		// odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
		// 	"/model/nemo/odometry", 10, std::bind(&NemoController::odomCallback, this, std::placeholders::_1));

		// camera_subscription = image_transport.subscribe(
		// 	"/camera", 1, std::bind(&NemoController::cameraCallback, this, std::placeholders::_1));

		// camera_info_subscription = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		// 	"/camera_info", 10, std::bind(&NemoController::cameraInfoCallback, this, std::placeholders::_1));

		moveToPosition(2, 3, 1.57);

		RCLCPP_INFO(this->get_logger(), "Initialized NEMO controller node");
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<NemoController>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}