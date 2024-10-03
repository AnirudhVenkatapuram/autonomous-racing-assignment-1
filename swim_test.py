#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

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

        # Define teleportation service proxy
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

        # Get user input directly for linear and angular velocities
        self.move_cmd = Twist()
        self.move_cmd.linear.x = float(input("Enter a linear velocity (2.0 to 6.0): "))
        self.move_cmd.angular.z = float(input("Enter an angular velocity (1.0 to 3.0): "))

        # Start the turtle movement
        rospy.loginfo(f"Starting figure-eight movement using linear velocity: {self.move_cmd.linear.x} "
                      f"and angular velocity: {self.move_cmd.angular.z}")

        # Start the loop to keep moving the turtle
        self.keep_moving()

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
        """ Teleport the turtle to the starting position. """
        # Use a service call to teleport the turtle to the starting position
        try:
            self.teleport_turtle(self.start_x, self.start_y, 0.0)  # Teleport to (start_x, start_y) with orientation 0.0
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
        PositionBasedFigureEight()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position-based figure-eight movement node terminated.")
