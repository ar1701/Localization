#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <memory>

// Define a controller node class inheriting from rclcpp::Node
class ControllerNode : public rclcpp::Node
{
public:
  // Constructor
  ControllerNode() : Node("controller_node"), goal_index(0), orientation_goal_reached(false),
                     distance_goal_reached(false), all_goals_reached(false)
  {
    // Declare parameters for x and y coordinates as empty vectors
    this->declare_parameter("x", std::vector<double>());
    this->declare_parameter("y", std::vector<double>());

    // Get x and y coordinates from parameters
    x_coordinates = this->get_parameter("x").as_double_array();
    y_coordinates = this->get_parameter("y").as_double_array();

    // Log information about the node being started and loaded file
    RCLCPP_INFO(this->get_logger(), "The node is started and file is loaded");

    // Get the number of goals from the loaded coordinates
    no_of_goals = x_coordinates.size();
    std::cout << "Number of goals: " << no_of_goals << std::endl;

    // If goals are provided, initialize the first goal
    if (no_of_goals > 0)
    {
      x_goal = x_coordinates[goal_index];
      y_goal = y_coordinates[goal_index];
    }
    else
    {
      // Log error if no goals are provided
      RCLCPP_ERROR(this->get_logger(), "No goals provided.");
      return;
    }

    // Initialize member variables
    x_current = 0.0;
    y_current = 0.0;
    yaw = 0.0;
    kp_angular = 0.5;
    kp_linear = 0.35;

    // Create TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create publisher for velocity commands
    velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // Create timer for periodic execution of goToGoal function
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&ControllerNode::goToGoal, this));
  }

private:
  // Function to set initial orientation towards the goal
  void setInitialOrientation(double x_goal, double y_goal, double x_current, double y_current, double yaw)
  {
    auto velocity_message = geometry_msgs::msg::Twist();
    double desired_angle_goal = std::atan2(y_goal - y_current, x_goal - x_current);
    double angle_difference =  desired_angle_goal - yaw;

    double angular_speed = angle_difference;
    
    if(angle_difference>360-angle_difference)
    {
    	angular_speed=angular_speed*-1;
    }
    
    if (std::fabs(desired_angle_goal - yaw) > 0.01)
    {
      velocity_message.angular.z = angular_speed * kp_angular;
      velocity_publisher->publish(velocity_message);
    }
    else
    {
      velocity_message.angular.z = 0.0;
      velocity_publisher->publish(velocity_message);
      orientation_goal_reached = true;
    }
  }

  // Function to move towards the goal
  void moveTowardsGoal(double x_goal, double y_goal, double x_current, double y_current, double yaw)
  {
    auto velocity_message = geometry_msgs::msg::Twist();
    double distance = std::abs(std::sqrt(std::pow((x_goal - x_current), 2) + std::pow((y_goal - y_current), 2)));
    double desired_angle_goal = std::atan2(y_goal - y_current, x_goal - x_current);
    double angle_difference = desired_angle_goal - yaw;

    double angular_speed = angle_difference;
    
    if(angle_difference>360-angle_difference)
    {
    	angular_speed=angular_speed*-1;
    }

    if (distance > 0.1)
    {
      velocity_message.linear.x = distance * kp_linear;
      velocity_message.angular.z = angular_speed * kp_angular;
      velocity_publisher->publish(velocity_message);
    }
    else
    {
      velocity_message.linear.x = 0.0;
      velocity_message.angular.z = 0.0;
      velocity_publisher->publish(velocity_message);
      distance_goal_reached = true;
    }
  }

  // Function to navigate to the goal
  void goToGoal()
  {
    std::string from_frame_rel = "base_link";
    std::string to_frame_rel = "odom";

    geometry_msgs::msg::TransformStamped t;
    try
    {
      t = tf_buffer_->lookupTransform(to_frame_rel, from_frame_rel, tf2::TimePointZero, std::chrono::milliseconds(100));
    }
    catch (const tf2::TransformException &ex)
    {
      auto velocity_message = geometry_msgs::msg::Twist();
      velocity_message.linear.x = 0.0;
      velocity_message.angular.z = 0.0;
      velocity_publisher->publish(velocity_message);

      // Log error if transformation cannot be performed
      RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          to_frame_rel.c_str(), from_frame_rel.c_str(), ex.what());
      return;
    }
    x_current = t.transform.translation.x;
    y_current = t.transform.translation.y;

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

    // Check if all goals are reached
    if (!all_goals_reached)
    {
      if (!orientation_goal_reached)
      {
        setInitialOrientation(x_goal, y_goal, x_current, y_current, yaw);
      }
      else if (!distance_goal_reached)
      {
        moveTowardsGoal(x_goal, y_goal, x_current, y_current, yaw);
      }
      else
      {
        // Log information about reaching the goal and fetch next goal
        RCLCPP_INFO(this->get_logger(), "Goal x: %f Y: %f reached.", x_goal, y_goal);
        RCLCPP_INFO(this->get_logger(), "Fetching next goal.........");
        goal_index++;
        RCLCPP_INFO(this->get_logger(), "Goal Number: %d", goal_index + 1);

        // Check if there are more goals to navigate
        if (goal_index < no_of_goals)
        {
          orientation_goal_reached = false;
          distance_goal_reached = false;
          x_goal = x_coordinates[goal_index];
          y_goal = y_coordinates[goal_index];
          RCLCPP_INFO(this->get_logger(), "New Goal x: %f New Goal Y: %f", x_goal, y_goal);
(std::chrono::seconds(1));
        }
        else
        {
          // Mark all goals as reached
          all_goals_reached = true;
          RCLCPP_INFO(this->get_logger(), "No Next Goal Found");
        }
      }
    }
    else
    {
      // Log once when all goals are completed
      RCLCPP_INFO_ONCE(this->get_logger(), "All goals completed");
    }
  }

  // Member variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> x_coordinates;
  std::vector<double> y_coordinates;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  double x_current, x_goal, y_current, y_goal, kp_angular, kp_linear, yaw;
  int no_of_goals, goal_index;
  bool orientation_goal_reached, distance_goal_reached, all_goals_reached;
};

// Main function
int main(int argc, char *argv[])
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  
  // Create and run the controller node
  rclcpp::spin(std::make_shared<ControllerNode>());
  
  // Shutdown ROS
  rclcpp::shutdown();
  
  return 0;
}

