#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute  # Import the teleport service

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

        # Define position threshold early to avoid attribute errors
        self.position_threshold = 0.05  # Small threshold for detecting the center

        # Center position coordinates of the turtlesim
        self.center_x = 5.5  # Initialize center_x attribute
        self.center_y = 5.5  # Initialize center_y attribute

        # Use default values for linear and angular velocities or command-line arguments
        self.move_cmd = Twist()
        # If input() is not available, use default values
        try:
            self.move_cmd.linear.x = self.get_user_input("Enter a linear velocity (2.0 to 6.0): ", 2.0, 6.0)
            self.move_cmd.angular.z = self.get_user_input("Enter an angular velocity (1.0 to 3.0): ", 1.0, 3.0)
        except EOFError:
            # Default values if input is not possible
            self.move_cmd.linear.x = 3.0
            self.move_cmd.angular.z = 1.5
            rospy.loginfo(f"Using default values: linear velocity = {self.move_cmd.linear.x}, "
                          f"angular velocity = {self.move_cmd.angular.z}")

        # Service client to teleport the turtle
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

        # Initialize last teleport time to prevent rapid teleportation
        self.last_teleport_time = rospy.Time.now()  # Initialize here

        # Start the turtle movement
        rospy.loginfo(f"Starting figure-eight movement using linear velocity: {self.move_cmd.linear.x} "
                      f"and angular velocity: {self.move_cmd.angular.z}")

        # Start the loop to keep moving the turtle
        self.keep_moving()

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

    def pose_callback(self, data):
        """ Callback function to get the current position of the turtle. """
        self.turtle_pose = data

        # Check if the turtle is near the center
        if self.is_near_center():
            # Check if enough time has passed since the last teleport to prevent frequent teleportation
            current_time = rospy.Time.now()
            if current_time - self.last_teleport_time > rospy.Duration(0.25):  # 0.25 seconds interval
                self.teleport_to_center()
                self.last_teleport_time = current_time  # Update the last teleport time

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

    def teleport_to_center(self):
        """ Teleport the turtle to the center of the screen. """
        rospy.loginfo(f"Teleporting turtle to center: x={self.center_x}, y={self.center_y}")
        try:
            # Teleport the turtle to the center position (5.5, 5.5) with no rotation (theta=0)
            self.teleport_turtle(self.center_x, self.center_y, 0.0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to teleport turtle to center: {e}")

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
