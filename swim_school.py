#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen  # Import service definitions
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

        # Disable the pen before teleporting to avoid drawing lines
        self.set_pen(0, 0, 0, 0, 1)  # Disable the pen
        self.teleport_turtle(5.5, 8.0)  # x=5.5, y=8.0 (Lower starting point)
        self.set_pen(255, 255, 255, 3, 0)  # Enable the pen back with default color and width

        # Store the initial position for resetting later
        self.initial_x = 5.5
        self.initial_y = 8.0
        self.initial_theta = 0

        # Calculate the duration based on angular velocity
        T = 2 * math.pi / angular_velocity  # Time for one full circle
        half_duration = T / 2  # Time for half of the figure-eight

        # Create the Twist message to control the turtle
        move_cmd = Twist()
        move_cmd.linear.x = linear_velocity

        # Start by making the first downward half-loop (half_duration)
        move_cmd.angular.z = angular_velocity  # Positive angular velocity to start the first arc
        self.publish_move_cmd_for_duration(move_cmd, half_duration)

        # Loop to move the turtle in a figure-eight pattern
        is_first_half_loop = True  # Track if we are doing the first half-loop or full loops
        loops_completed = 0  # Counter to track completed loops

        while not rospy.is_shutdown():
            if is_first_half_loop:
                # After the first half-loop, change direction to complete the first full loop
                move_cmd.angular.z = -move_cmd.angular.z  # Reverse angular direction
                self.publish_move_cmd_for_duration(move_cmd, half_duration)  # Complete the first loop
                is_first_half_loop = False  # Switch to full loop mode after the first full loop
                loops_completed += 1
            else:
                # Continue switching direction every full duration (T) to maintain a figure-eight
                move_cmd.angular.z = -move_cmd.angular.z  # Reverse angular direction
                self.publish_move_cmd_for_duration(move_cmd, T)  # Complete a full loop
                loops_completed += 1

            # Reset turtle position every 2 full loops to avoid drift
            if loops_completed == 2:
                self.set_pen(0, 0, 0, 0, 1)  # Disable the pen before teleporting
                self.teleport_turtle(self.initial_x, self.initial_y, self.initial_theta)  # Reset to initial position
                self.set_pen(255, 255, 255, 3, 0)  # Enable the pen back
                loops_completed = 0  # Reset the counter after a complete figure-eight

    def set_pen(self, r, g, b, width, off):
        """ Set the pen color, width, and turn it on/off. """
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            pen_service = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            pen_service(r, g, b, width, off)
            rospy.loginfo(f"Pen set: color=({r}, {g}, {b}), width={width}, off={off}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def teleport_turtle(self, x, y, theta=0):
        """ Teleport the turtle to a specific location with orientation (default is 0). """
        rospy.wait_for_service('/turtle1/teleport_absolute')
        try:
            teleport_service = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
            teleport_service(x, y, theta)
            rospy.loginfo(f"Turtle teleported to x: {x}, y: {y}, theta: {theta}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

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
