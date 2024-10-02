#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

class FigureEightMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('figure_eight_mover', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Set the rate at which to send commands (20 Hz)
        rate = rospy.Rate(20)  # 20 Hz

        # Create the Twist message to control the turtle
        move_cmd = Twist()

        # Set linear and initial angular velocities
        linear_velocity = 2.0
        angular_velocity = 1.0

        # Set initial velocities
        move_cmd.linear.x = linear_velocity
        move_cmd.angular.z = angular_velocity

        # Time required to complete one full circle
        T = 2 * math.pi / angular_velocity

        # Counter to keep track of completed circles
        circles_completed = 0

        # Loop to move the turtle
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()

            # Publish the move command to make the turtle move in a circle
            self.cmd_vel.publish(move_cmd)

            # Check if one full circle is completed (elapsed_time >= T)
            if elapsed_time >= T:
                # Switch angular velocity to reverse direction
                move_cmd.angular.z = -move_cmd.angular.z
                # Reset the start time for the next circle
                start_time = rospy.Time.now()
                # Increment the circle counter
                circles_completed += 1

                # Log the completion of a circle
                rospy.loginfo(f"Completed circles: {circles_completed}")

            # After completing two circles, continue switching back and forth to maintain the figure-eight pattern
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the turtle.")
        # Stop the turtle by sending an empty Twist message
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        FigureEightMover()
    except rospy.ROSInterruptException:
        rospy.loginfo("Figure-eight movement node terminated.")
