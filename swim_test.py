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

        # Set the rate at which to send commands (100 Hz)
        rate = rospy.Rate(100)  # 100 Hz

        # Create the Twist message to control the turtle
        self.move_cmd = Twist()
        stop_cmd = Twist()  # A Twist message to stop the turtle

        # Set initial linear and angular velocities
        self.move_cmd.linear.x = 2.0
        self.move_cmd.angular.z = 1.0

        # High precision value of Pi
        pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899

        # Time required to complete one full circle using high precision pi
        self.T = 2 * pi / self.move_cmd.angular.z

        # Counter to keep track of completed circles
        self.circles_completed = 0

        # Loop to move the turtle
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()

            # Publish the move command to make the turtle move in a circle
            self.cmd_vel.publish(self.move_cmd)

            # Check if one full circle is completed (elapsed_time >= T)
            if elapsed_time >= self.T:
                # Stop the turtle briefly
                rospy.loginfo(f"Completed {self.circles_completed + 1} circle(s). Stopping briefly...")
                self.cmd_vel.publish(stop_cmd)  # Stop the turtle
                rospy.sleep(0.1)  # Wait for 0.1 seconds

                # Switch angular velocity to reverse direction
                self.move_cmd.angular.z = -self.move_cmd.angular.z

                # Increment the circle counter
                self.circles_completed += 1

                # Reset the start time for the next circle
                start_time = rospy.Time.now()

                # Log the completion of a circle and direction switch
                rospy.loginfo(f"Switching direction. Circles completed: {self.circles_completed}")

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
