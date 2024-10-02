#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute
from std_srvs.srv import Empty
import time

class SquareDrawer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('back_to_square_one', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Define services for setting the background color and pen
        rospy.wait_for_service('/clear')
        rospy.wait_for_service('/turtle1/set_pen')
        rospy.wait_for_service('/turtle1/teleport_absolute')

        # Create service proxies
        self.clear_bg = rospy.ServiceProxy('/clear', Empty)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

        # Get user input for the square's side length
        self.side_length = self.get_user_input("Enter the length of the side of the square (1 to 5): ", 1.0, 5.0)

        # Change the background color to red
        self.change_background_to_red()

        # Teleport the turtle to the starting position (bottom-left corner of the square)
        self.teleport_to_start_position()

        # Draw the square with the specified side length
        self.draw_square(self.side_length)

        # Stop the turtle after drawing the square
        self.stop_turtle()

    def get_user_input(self, prompt, min_val, max_val):
        """ Get a valid user input between min_val and max_val. """
        while True:
            try:
                value = float(input(prompt))
                if min_val <= value <= max_val:
                    return value
                else:
                    print(f"Please enter a value between {min_val} and {max_val}.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")

    def change_background_to_red(self):
        """ Change the ocean background color to red. """
        rospy.loginfo("Changing the background color to red...")
        self.set_pen(255, 0, 0, 3, 1)  # Change pen color to red (255, 0, 0)
        self.clear_bg()  # Clear the screen with the new background color

    def teleport_to_start_position(self):
        """ Teleport the turtle to the starting position at (1, 1). """
        rospy.loginfo("Teleporting turtle to start position (1, 1)...")
        self.teleport_turtle(1.0, 1.0, 0)  # Teleport to position (1, 1) with theta=0

    def draw_square(self, side_length):
        """ Draw a square with the given side length. """
        # Create a Twist message to control the turtle
        move_cmd = Twist()

        # Set the linear speed and the time to travel one side
        linear_speed = 1.0  # Linear speed in units/second
        move_cmd.linear.x = linear_speed

        # Time to travel one side of the square
        side_time = side_length / linear_speed

        # Set the angular speed and time to make a 90 degree turn
        angular_speed = 1.0  # Angular speed in radians/second
        move_cmd.angular.z = 0.0  # No angular movement initially

        rate = rospy.Rate(10)  # 10 Hz rate

        for _ in range(4):  # Draw 4 sides of the square
            # Move forward to draw one side of the square
            move_cmd.linear.x = linear_speed
            move_cmd.angular.z = 0.0
            rospy.loginfo(f"Drawing side with length {side_length}")
            t0 = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - t0) < side_time:
                self.cmd_vel.publish(move_cmd)
                rate.sleep()

            # Stop briefly before turning
            move_cmd.linear.x = 0.0
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(0.1)  # Pause for a short moment before turning

            # Make a 90 degree turn
            move_cmd.angular.z = angular_speed  # Rotate at a constant angular speed
            turn_time = 1.5708 / angular_speed  # 90 degrees = π/2 = 1.5708 radians
            t0 = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - t0) < turn_time:
                self.cmd_vel.publish(move_cmd)
                rate.sleep()

            # Stop briefly after turning
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(0.1)  # Pause briefly after each turn

    def stop_turtle(self):
        """ Stop the turtle after drawing the square. """
        rospy.loginfo("Stopping the turtle...")
        stop_cmd = Twist()  # Create a zero-velocity Twist message
        self.cmd_vel.publish(stop_cmd)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        SquareDrawer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Square drawing node terminated.")
