#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute  # Import the service definition for teleportation
from turtlesim.msg import Pose

class TeleportFigureEightMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('teleport_figure_eight_mover', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Create a subscriber to the /turtle1/pose topic to get the turtle's position
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Initialize position variables
        self.turtle_pose = Pose()
        self.reached_center = False  # Keeps track of whether the turtle is near the center

        # Create the Twist message to control the turtle
        self.move_cmd = Twist()

        # Set initial linear and angular velocities
        self.move_cmd.linear.x = 2.0  # Linear velocity
        self.move_cmd.angular.z = 1.0  # Angular velocity

        # Center position coordinates of the turtlesim
        self.center_x = 5.5
        self.center_y = 5.5

        # Threshold to determine if turtle is close to the center
        self.position_threshold = 0.1

        # Track the circle being completed (top or bottom)
        self.completing_top_circle = True  # Start with the top circle

        # Start the turtle movement
        rospy.loginfo("Starting teleport-based figure-eight movement...")

        # Start the loop to keep moving the turtle
        self.keep_moving()

    def pose_callback(self, data):
        """ Callback function to get the current position of the turtle. """
        self.turtle_pose = data

        # Check if the turtle is near the center
        if self.is_near_center():
            # If the turtle is near the center, teleport it back and switch direction
            if not self.reached_center:
                self.reached_center = True  # Indicate that the turtle has reached the center
                self.teleport_to_center()  # Teleport to the center and adjust orientation
                self.switch_direction()    # Change direction to form the next circle
        else:
            # Reset the flag when the turtle is not at the center
            self.reached_center = False

    def is_near_center(self):
        """ Check if the turtle is near the center of the screen. """
        return abs(self.turtle_pose.x - self.center_x) < self.position_threshold and \
               abs(self.turtle_pose.y - self.center_y) < self.position_threshold

    def teleport_to_center(self):
        """ Teleport the turtle to the center of the screen and set its orientation. """
        rospy.wait_for_service('/turtle1/teleport_absolute')
        try:
            teleport_service = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

            # Determine the orientation to set based on the circle being completed
            if self.completing_top_circle:
                # If completing the top circle, face left (theta = pi or 180 degrees)
                theta = 3.14159
            else:
                # If completing the bottom circle, face right (theta = 0 degrees)
                theta = 0.0

            # Teleport the turtle to the center with the calculated orientation
            teleport_service(self.center_x, self.center_y, theta)
            rospy.loginfo(f"Turtle teleported to center. New orientation: {theta} radians.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def switch_direction(self):
        """ Switch the direction of the turtle's angular velocity to form the figure-eight pattern. """
        # Reverse the direction of angular velocity
        self.move_cmd.angular.z = -self.move_cmd.angular.z

        # Log the switch event and update the circle being completed
        if self.completing_top_circle:
            rospy.loginfo("Switched direction to complete bottom circle.")
            self.completing_top_circle = False  # Now complete the bottom circle
        else:
            rospy.loginfo("Switched direction to complete top circle.")
            self.completing_top_circle = True  # Now complete the top circle

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
        TeleportFigureEightMover()
    except rospy.ROSInterruptException:
        rospy.loginfo("Teleport-based figure-eight movement node terminated.")
