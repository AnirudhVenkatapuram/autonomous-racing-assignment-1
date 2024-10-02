#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math  # Import math to use pi

class SwimSchool:

    def __init__(self):
        rospy.init_node('swim_school', anonymous=False)

        # Message to screen
        rospy.loginfo(" Press CTRL+c to stop the turtle")

        rospy.on_shutdown(self.shutdown)

        # Publisher to send movement commands to the turtle
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Set the rate at which to send commands (10 Hz)
        rate = rospy.Rate(10)

        # Get user inputs for linear and angular velocities
        linear_velocity = self.get_user_input("Enter a linear velocity (x) between 2 and 6: ", 2.0, 6.0)
        angular_velocity = self.get_user_input("Enter an angular velocity (z) between 1 and 3: ", 1.0, 3.0)

        rospy.loginfo(f"Linear velocity set to {linear_velocity}")
        rospy.loginfo(f"Angular velocity set to {angular_velocity}")

        # Calculate the duration based on angular velocity
        # Duration to complete one full circle is T = 2pi/angular_velocity
        T = 2 * math.pi / angular_velocity

        # We want the turtle to complete each half of the figure-eight in T/2 time
        half_duration = T / 2

        # Create the Twist message to control the turtle
        move_cmd = Twist()
        move_cmd.linear.x = linear_velocity

        # Loop to move the turtle in a figure-eight pattern
        while not rospy.is_shutdown():
            # First part of the figure-eight (left half)
            move_cmd.angular.z = angular_velocity  # Positive for counter-clockwise
            self.publish_move_cmd_for_duration(move_cmd, half_duration)

            # Second part of the figure-eight (right half)
            move_cmd.angular.z = -angular_velocity  # Negative for clockwise
            self.publish_move_cmd_for_duration(move_cmd, half_duration)

    def publish_move_cmd_for_duration(self, move_cmd, duration):
        """ Helper function to publish move command for a specific duration in seconds. """
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)  # 10 Hz rate

        while rospy.Time.now() < end_time:
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

    def get_user_input(self, prompt, min_val, max_val):
        """ Helper function to get a valid user input between min_val and max_val. """
        while True:
            try:
                value = float(input(prompt))
                if min_val <= value <= max_val:
                    return value
                else:
                    print(f"Please enter a value between {min_val} and {max_val}.")
            except ValueError:
                print("Invalid input, please enter a numeric value.")

    def shutdown(self):
        rospy.loginfo("Stopping the turtle")

        # Stop the turtle by sending an empty Twist message
        self.cmd_vel.publish(Twist())

        # Sleep to make sure the turtle stops
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        SwimSchool()
    except rospy.ROSInterruptException:
        rospy.loginfo("Swim school node terminated.")
