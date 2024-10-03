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
        self.reached_start = False  # Keeps track of whether the turtle is near the start

        # Define position threshold for teleportation
        self.position_threshold = 0.1  # Small threshold for detecting the start

        # Define the starting position of the turtle
        self.start_x = 5.5
        self.start_y = 5.5

        # Keep track of the last teleportation time to avoid frequent teleports
        self.last_teleport_time = rospy.Time.now()  # Store the last teleport time

        # Get user input for linear and angular velocities
        self.move_cmd = Twist()

        # Try to get user input; if an EOFError occurs, fallback to default values
        try:
            self.move_cmd.linear.x = self.get_user_input("Enter a linear velocity (2.0 to 6.0): ", 2.0, 6.0)
            self.move_cmd.angular.z = self.get_user_input("Enter an angular velocity (1.0 to 3.0): ", 1.0, 3.0)
        except EOFError:
            # Default values if input is not possible or EOFError occurs
            self.move_cmd.linear.x = 3.0
            self.move_cmd.angular.z = 1.5
            rospy.loginfo(f"Using default values: linear velocity = {self.move_cmd.linear.x}, "
                          f"angular velocity = {self.move_cmd.angular.z}")

        # Start the turtle movement
        rospy.loginfo(f"Starting figure-eight movement using linear velocity: {self.move_cmd.linear.x} "
                      f"and angular velocity: {self.move_cmd.angular.z}")

        # Start the loop to keep moving the turtle
        self.keep_moving()

    def get_user_input(self, prompt, min_val, max_val):
        """ Get a valid user input between min_val and max_val. """
        while True:
            try:
                # Get user input from the terminal
                value = float(input(prompt))
                if min_val <= value <= max_val:
                    return value
                else:
                    print(f"Please enter a value between {min_val} and {max_val}.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")
            except EOFError:
                # This will catch the EOFError and handle it gracefully
                print("No input detected, using default values.")
                raise EOFError

    def pose_callback(self, data):
        """ Callback function to get the current position of the turtle. """
        self.turtle_pose = data

        # Check if the turtle is near the start position
        if self.is_near_start():
            # Check if enough time has passed since the last teleport to prevent frequent teleportation
            current_time = rospy.Time.now()
            if current_time - self.last_teleport_time > rospy.Duration(0.25):  # Only teleport if at least 0.25 seconds have passed
                rospy.loginfo(f"Teleporting turtle to start position: x={self.start_x}, y={self.start_y}")
                self.last_teleport_time = current_time  # Update the last teleport time

    def is_near_start(self):
        """ Check if the turtle is near the start position of the circle. """
        return abs(self.turtle_pose.x - self.start_x) < self.position_threshold and \
               abs(self.turtle_pose.y - self.start_y) < self.position_threshold

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
