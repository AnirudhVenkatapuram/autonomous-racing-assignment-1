#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

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

        # High precision value of Pi with many decimal points
        pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899

        # Time required to complete one full circle using high precision pi
        T = 2 * pi / angular_velocity

        # Counter to keep track of completed circles
        circles_completed = 0

        # Loop to move the turtle
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()

            # Publish the move command to make the turtle move in a circle
            self.cmd_vel.publish(move_cmd)

            # Check if one full circle is completed (add a small buffer to the threshold)
            if elapsed_time >= T + 0.05:  # Adjust the threshold slightly to ensure a complete circle
                # Switch angular velocity to reverse direction
                move_cmd.angular.z = -move_cmd.angular.z
                # Reset the start time for the next circle
                start_time = rospy.Time.now()
                # Increment the circle counter
                circles_completed += 1

                # Log the completion of a circle
                rospy.loginfo(f"Completed circles: {circles_completed}")

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
