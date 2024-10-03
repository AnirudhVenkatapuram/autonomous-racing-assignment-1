#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class SwimToGoal:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('swim_to_goal', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Define a subscriber to the /turtle1/pose topic
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        # Initialize the turtle's pose
        self.turtle_pose = Pose()
        self.rate = rospy.Rate(10)  # 10 Hz rate for publishing commands

        # Gain constants for proportional control
        self.V_gain = 1.0  # Gain for linear velocity
        self.A_gain = 4.0  # Gain for angular velocity

        # Flag to indicate when to stop the turtle
        self.goal_reached = False

        # Get user input for the goal position and tolerance
        self.goal_x = float(input("Set your x goal: "))
        self.goal_y = float(input("Set your y goal: "))
        self.tolerance = float(input("Set your tolerance: "))

        # Run the turtle control loop to swim to the goal
        self.swim_to_goal()

    def update_pose(self, data):
        """Callback function which updates the current pose of the turtle."""
        self.turtle_pose = data

    def euclidean_distance(self, goal_x, goal_y):
        """Calculate Euclidean distance between current position and goal position."""
        return math.sqrt(pow((goal_x - self.turtle_pose.x), 2) + pow((goal_y - self.turtle_pose.y), 2))

    def linear_velocity(self, goal_x, goal_y):
        """Calculate linear velocity proportional to the distance to the goal."""
        return self.V_gain * self.euclidean_distance(goal_x, goal_y)

    def steering_angle(self, goal_x, goal_y):
        """Calculate the desired steering angle to reach the goal."""
        return math.atan2(goal_y - self.turtle_pose.y, goal_x - self.turtle_pose.x)

    def angular_velocity(self, goal_x, goal_y):
        """Calculate angular velocity proportional to the difference in heading."""
        return self.A_gain * (self.steering_angle(goal_x, goal_y) - self.turtle_pose.theta)

    def swim_to_goal(self):
        """Swim the turtle to the goal position using proportional control."""
        move_cmd = Twist()

        while not rospy.is_shutdown() and not self.goal_reached:
            # Calculate the Euclidean distance to the goal
            distance = self.euclidean_distance(self.goal_x, self.goal_y)

            # If the distance is within the tolerance, stop the turtle
            if distance < self.tolerance:
                rospy.loginfo("Goal reached!")
                self.stop_turtle()
                self.goal_reached = True
                break

            # Set linear velocity proportional to the distance
            move_cmd.linear.x = self.linear_velocity(self.goal_x, self.goal_y)
            move_cmd.angular.z = self.angular_velocity(self.goal_x, self.goal_y)

            # Publish the velocity command to move the turtle
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

    def stop_turtle(self):
        """Stop the turtle's movement."""
        rospy.loginfo("Stopping the turtle.")
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)

if __name__ == '__main__':
    try:
        SwimToGoal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
