#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationNode : public rclcpp::Node {
  private:
	rclcpp::Publisher<PoseStamped>::SharedPtr goal_publisher;
	rclcpp_action::Client<NavigateToPose>::SharedPtr action_client;

	// Callback functions for goal response and result
	void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
		if (!goal_handle)
			RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server");

		else
			RCLCPP_INFO(get_logger(), "Goal accepted by the action server");
	}

	void result_callback(const GoalHandleNavigateToPose::WrappedResult &result) {
		switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_INFO(get_logger(), "Goal was successful");
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(get_logger(), "Goal was aborted");
			break;
		default:
			RCLCPP_ERROR(get_logger(), "Unknown result code");
			break;
		}
	}

  public:
	NavigationNode() : rclcpp::Node("navigation_node") {
		// Create the publisher to send goal poses
		goal_publisher = create_publisher<PoseStamped>("goal_pose", 10);

		// Create the action client for NavigateToPose
		action_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

		RCLCPP_INFO(this->get_logger(), "Initialized navigation node");
	}

	void send_goal(const double x, const double y, const double yaw) {
		using namespace std::placeholders;

		auto goal_msg = std::make_shared<PoseStamped>();

		// Fill in the goal pose
		goal_msg->header.frame_id = "map";
		goal_msg->pose.position.x = x;
		goal_msg->pose.position.y = y;
		// TODO: Fill in the orientation

		// Publish the goal pose
		goal_publisher->publish(*goal_msg);

		// Send goal to BasicNavigator using NavigateToPose action
		auto goal = NavigateToPose::Goal();
		goal.pose = *goal_msg;

		// Send the goal to the action server
		RCLCPP_INFO(this->get_logger(), "Sending goal");

		auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

		send_goal_options.goal_response_callback = std::bind(&NavigationNode::goal_response_callback, this, _1);
		send_goal_options.result_callback = std::bind(&NavigationNode::result_callback, this, _1);

		action_client->async_send_goal(goal, send_goal_options);

		RCLCPP_INFO(this->get_logger(), "Moving to (%.2f, %.2f)", x, y);
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<NavigationNode>();

	node->send_goal(2.0, 3.0, 1.57);

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}