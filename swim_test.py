#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class PositionBasedFigureEight:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('position_based_figure_eight', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Create a subscriber to the /turtle1/pose topic to get the turtle's position
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Initialize position variables
        self.turtle_pose = Pose()
        self.reached_center = False  # Keeps track of whether the turtle is near the center

        # Create the Twist message to control the turtle
        self.move_cmd = Twist()

        # Set initial linear and angular velocities
        self.move_cmd.linear.x = 2.0  # Linear velocity
        self.move_cmd.angular.z = 1.0  # Angular velocity

        # Center position coordinates of the turtlesim
        self.center_x = 5.5
        self.center_y = 5.5

        # Threshold to determine if turtle is close to the center
        self.position_threshold = 0.05

        # Start the turtle movement
        rospy.loginfo("Starting figure-eight movement using position-based switching...")

        # Start the loop to keep moving the turtle
        self.keep_moving()

    def pose_callback(self, data):
        """ Callback function to get the current position of the turtle. """
        self.turtle_pose = data

        # Check if the turtle is near the center
        if self.is_near_center():
            # If the turtle is near the center, switch direction if it hasn't already
            if not self.reached_center:
                self.switch_direction()
                self.reached_center = True  # Indicate that the turtle has reached the center
        else:
            # Reset the flag when the turtle is not at the center
            self.reached_center = False

    def is_near_center(self):
        """ Check if the turtle is near the center of the screen. """
        return abs(self.turtle_pose.x - self.center_x) < self.position_threshold and \
               abs(self.turtle_pose.y - self.center_y) < self.position_threshold

    def switch_direction(self):
        """ Switch the direction of the turtle's angular velocity to form the figure-eight pattern. """
        # Log the switch event
        rospy.loginfo(f"Switching direction at position: x={self.turtle_pose.x}, y={self.turtle_pose.y}")

        # Reverse the direction of angular velocity
        self.move_cmd.angular.z = -self.move_cmd.angular.z

    def keep_moving(self):
        """ Keep publishing the move command indefinitely. """
        rate = rospy.Rate(100)  # Increase rate to 100 Hz for smoother control
        while not rospy.is_shutdown():
            # Publish the move command to make the turtle move
            self.cmd_vel.publish(self.move_cmd)
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the turtle.")
        # Stop the turtle by sending an empty Twist message
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        PositionBasedFigureEight()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position-based figure-eight movement node terminated.")
