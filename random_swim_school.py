#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
import random  # Import the random module to generate random values

class RandomPositionFigureEight:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('random_position_figure_eight', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Create a subscriber to the /turtle1/pose topic to get the turtle's position
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Initialize position variables
        self.turtle_pose = Pose()
        self.reached_start = False  # Keeps track of whether the turtle is near the start

        # Define position threshold for teleportation
        self.position_threshold = 0.1  # Small threshold for detecting the start

        # Generate random starting position for the turtle (between 3 and 8 for both x and y)
        self.start_x = random.uniform(3.0, 8.0)
        self.start_y = random.uniform(3.0, 8.0)

        # Generate random linear and angular velocities
        self.move_cmd = Twist()
        self.move_cmd.linear.x = random.uniform(2.0, 6.0)  # Random linear velocity between 2 and 6
        self.move_cmd.angular.z = random.uniform(1.0, 3.0)  # Random angular velocity between 1 and 3

        # Initialize the last teleport time to avoid frequent teleportation
        self.last_teleport_time = rospy.Time.now()

        # Teleport the turtle to the random starting position before starting
        self.teleport_to_position(self.start_x, self.start_y)

        # Print the random values to the terminal
        rospy.loginfo(f"Starting figure-eight pattern from random position: x={self.start_x}, y={self.start_y}")
        rospy.loginfo(f"Random linear velocity: {self.move_cmd.linear.x}, Random angular velocity: {self.move_cmd.angular.z}")

        # Start the turtle movement
        self.keep_moving()

    def set_pen_state(self, r, g, b, width, off):
        """ Set the pen state of the turtle. """
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            set_pen(r, g, b, width, off)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set pen state: {e}")

    def pose_callback(self, data):
        """ Callback function to get the current position of the turtle. """
        self.turtle_pose = data

        # Check if the turtle is near the start position
        if self.is_near_start():
            # Check if enough time has passed since the last teleport to prevent frequent teleportation
            current_time = rospy.Time.now()
            if current_time - self.last_teleport_time > rospy.Duration(0.25):  # Only teleport if at least 0.25 seconds have passed
                self.last_teleport_time = current_time  # Update the last teleport time
                # Change the angular velocity direction right before teleporting
                self.move_cmd.angular.z = -self.move_cmd.angular.z
                self.teleport_to_start()  # Teleport the turtle to the starting position

    def is_near_start(self):
        """ Check if the turtle is near the start position of the circle. """
        return abs(self.turtle_pose.x - self.start_x) < self.position_threshold and \
               abs(self.turtle_pose.y - self.start_y) < self.position_threshold

    def teleport_to_start(self):
        """ Teleport the turtle back to the starting position. """
        self.teleport_to_position(self.start_x, self.start_y)

    def teleport_to_position(self, x, y):
        """ Teleport the turtle to a specified position (x, y). """
        # Turn off the pen to avoid drawing a line during teleportation
        self.set_pen_state(255, 255, 255, 1, 1)  # Set pen state to off (use any color and width)

        # Wait for the teleport service to become available
        rospy.wait_for_service('/turtle1/teleport_absolute')
        try:
            # Create a service proxy for the teleport_absolute service
            teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
            teleport_turtle(x, y, 0.0)  # Teleport turtle to (x, y) with theta = 0.0

            # After teleportation, turn on the pen again with the default color (black)
            self.set_pen_state(0, 0, 0, 2, 0)  # Default black color with pen width 2
        except rospy.ServiceException as e:
            rospy.logerr(f"Teleportation failed: {e}")

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
        RandomPositionFigureEight()
    except rospy.ROSInterruptException:
        rospy.loginfo("Random position figure-eight movement node terminated.")
