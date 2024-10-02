#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class FigureEightMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('figure_eight_mover', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Create a subscriber to the /turtle1/pose topic to get the turtle's position
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Initialize position variables
        self.turtle_pose = Pose()

        # Create the Twist message to control the turtle
        self.move_cmd = Twist()

        # Set initial linear and angular velocities
        self.move_cmd.linear.x = 2.0
        self.move_cmd.angular.z = 1.0

        # Set flags to track which part of the figure-eight we're in
        self.switch_to_bottom_circle = False
        self.switch_to_top_circle = False

        # Start the turtle movement
        rospy.loginfo("Starting figure-eight movement...")

        # Start the loop to keep moving the turtle
        self.keep_moving()

    def pose_callback(self, data):
        """ Callback function to get the current position of the turtle. """
        self.turtle_pose = data

        # Check for position thresholds to switch direction
        self.check_position_and_switch()

    def check_position_and_switch(self):
        """ Check the turtle's position and switch direction if necessary. """
        # Get the current position of the turtle
        x = self.turtle_pose.x
        y = self.turtle_pose.y

        # If the turtle is at the top of the figure-eight (near y=10), switch to bottom circle
        if y >= 10.5 and not self.switch_to_bottom_circle:
            self.move_cmd.angular.z = -self.move_cmd.angular.z
            self.switch_to_bottom_circle = True
            self.switch_to_top_circle = False
            rospy.loginfo("Switched to bottom circle. Current position: x={}, y={}".format(x, y))

        # If the turtle is at the bottom of the figure-eight (near y=2), switch to top circle
        if y <= 1.5 and not self.switch_to_top_circle:
            self.move_cmd.angular.z = -self.move_cmd.angular.z
            self.switch_to_top_circle = True
            self.switch_to_bottom_circle = False
            rospy.loginfo("Switched to top circle. Current position: x={}, y={}".format(x, y))

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
        FigureEightMover()
    except rospy.ROSInterruptException:
        rospy.loginfo("Figure-eight movement node terminated.")
