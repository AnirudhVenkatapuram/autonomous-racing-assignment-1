#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

class CircleWithTeleport:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('circle_with_teleport', anonymous=False)

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

        # Get user inputs for linear and angular velocities
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.get_user_input("Enter a linear velocity (2.0 to 6.0): ", 2.0, 6.0)
        self.move_cmd.angular.z = self.get_user_input("Enter an angular velocity (1.0 to 3.0): ", 1.0, 3.0)

        # Initialize the teleportation service
        rospy.loginfo("Waiting for the /turtle1/teleport_absolute service to be available...")
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        rospy.loginfo("/turtle1/teleport_absolute service is now available.")

        # Start the turtle movement
        rospy.loginfo(f"Starting circle movement with linear velocity: {self.move_cmd.linear.x} "
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

        # Check if the turtle is near the start position
        if self.is_near_start():
            # Check if enough time has passed since the last teleport to prevent frequent teleportation
            current_time = rospy.Time.now()
            if current_time - self.last_teleport_time > rospy.Duration(0.25):
                # Switch the angular velocity to the negative value right before teleporting
                self.switch_angular_velocity()
                
                # Teleport to the start position
                self.teleport_to_start()
                self.last_teleport_time = current_time  # Update the last teleport time

    def is_near_start(self):
        """ Check if the turtle is near the start position of the circle. """
        return abs(self.turtle_pose.x - self.start_x) < self.position_threshold and \
               abs(self.turtle_pose.y - self.start_y) < self.position_threshold

    def switch_angular_velocity(self):
        """ Switch the direction of the turtle's angular velocity. """
        rospy.loginfo(f"Switching angular velocity from {self.move_cmd.angular.z} to {-self.move_cmd.angular.z}")
        self.move_cmd.angular.z = -self.move_cmd.angular.z  # Reverse the angular velocity

    def teleport_to_start(self):
        """ Teleport the turtle to the starting position of the circle. """
        rospy.loginfo(f"Teleporting turtle to start position: x={self.start_x}, y={self.start_y}")
        try:
            # Teleport the turtle to the start position (5.5, 5.5) with no rotation (theta=0)
            self.teleport_turtle(self.start_x, self.start_y, 0.0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to teleport turtle to start: {e}")

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
        CircleWithTeleport()
    except rospy.ROSInterruptException:
        rospy.loginfo("Circle movement node terminated.")
