#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

class FigureEightMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('figure_eight_mover', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # High precision value of Pi
        pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899

        # Set linear and initial angular velocities
        linear_velocity = 2.0
        angular_velocity = 1.0

        # Time required to complete one full circle using high precision pi
        self.T = 2 * pi / angular_velocity

        # Create the Twist message to control the turtle
        self.move_cmd = Twist()
        self.move_cmd.linear.x = linear_velocity
        self.move_cmd.angular.z = angular_velocity

        # Set initial variables
        self.circles_completed = 0

        # Timer to control direction switching
        rospy.Timer(rospy.Duration(self.T), self.switch_direction)

        rospy.loginfo("Starting figure-eight movement...")

        # Keep the turtle moving indefinitely
        self.keep_moving()

    def switch_direction(self, event):
        """ Switches the direction of angular velocity to create the figure-eight pattern. """
        # Switch angular velocity to reverse direction
        self.move_cmd.angular.z = -self.move_cmd.angular.z
        self.circles_completed += 1
        rospy.loginfo(f"Switching direction. Circles completed: {self.circles_completed}")

    def keep_moving(self):
        """ Publishes the current movement command indefinitely. """
        rate = rospy.Rate(100)  # 20 Hz rate

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
