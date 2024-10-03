#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative
from std_srvs.srv import Empty

class SquareDrawer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('back_to_square_one', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Define services for setting the background color, clearing the screen, and teleporting
        rospy.wait_for_service('/clear')
        rospy.wait_for_service('/turtle1/set_pen')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        rospy.wait_for_service('/turtle1/teleport_relative')

        # Create service proxies
        self.clear_bg = rospy.ServiceProxy('/clear', Empty)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.teleport_relative = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)

        # Get user input for the square's side length
        self.side_length = self.get_user_input("Enter the length of the side of the square (1 to 5): ", 1.0, 5.0)

        # Change the background color to red immediately after receiving user input
        self.change_background_color_to_red()

        # Teleport the turtle to the starting position (bottom-left corner of the square) without drawing any lines
        self.teleport_to_start_position()

        # Set the pen color and enable drawing before starting
        self.set_pen_state(0, 0, 0, 3, 0)  # Pen color: Black, width: 3, pen enabled (off=0)

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

    def change_background_color_to_red(self):
        """ Change the ocean background color to red using ROS parameters. """
        rospy.loginfo("Changing the background color to red using ROS parameters...")

        # Set the background color parameters on the ROS parameter server
        rospy.set_param('/turtlesim/background_r', 255)  # Red value
        rospy.set_param('/turtlesim/background_g', 0)    # Green value
        rospy.set_param('/turtlesim/background_b', 0)    # Blue value

        # Clear the screen to apply the new background color
        try:
            self.clear_bg()
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear screen for background color change: {e}")

    def teleport_to_start_position(self):
        """ Teleport the turtle to the starting position at (1, 1) without drawing lines. """
        rospy.loginfo("Teleporting turtle to start position (1, 1) without drawing lines...")

        # Disable the pen before teleporting to avoid drawing lines
        self.set_pen_state(255, 255, 255, 5, 1)  # Disable pen (off=1) with an arbitrary color

        try:
            # Teleport to position (1, 1) with orientation (theta) = 0 radians
            self.teleport_turtle(1.0, 1.0, 0.0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to teleport turtle: {e}")

        # Re-enable the pen after teleporting to start drawing
        self.set_pen_state(0, 0, 0, 3, 0)  # Enable pen (off=0) with black color and width 3

    def set_pen_state(self, r, g, b, width, off):
        """ Set the pen state to control drawing lines.
            r, g, b: Color of the pen.
            width: Width of the pen line.
            off: 1 to disable the pen, 0 to enable. """
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            set_pen(r, g, b, width, off)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call set_pen service: {e}")

    def draw_square(self, side_length):
        """ Draw a square with the given side length. """
        # Create a Twist message to control the turtle
        move_cmd = Twist()

        # Set the linear speed and the time to travel one side
        linear_speed = 1.0  # Linear speed in units/second
        move_cmd.linear.x = linear_speed

        # Time to travel one side of the square
        side_time = side_length / linear_speed

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

            # Stop the turtle before making a turn
            move_cmd.linear.x = 0.0
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(0.1)  # Pause briefly before turning

            # Use teleport_relative to make a precise 90 degree turn
            try:
                self.teleport_relative(0, 1.5708)  # 90 degrees = π/2 = 1.5708 radians
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to teleport turtle for turning: {e}")
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
