#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class CircleMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('circle_mover', anonymous=False)

        # Define a publisher to the /turtle1/cmd_vel topic
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Set the rate at which to send commands (20 Hz)
        rate = rospy.Rate(20)  # 20 Hz

        # Create the Twist message to control the turtle
        move_cmd = Twist()

        # Set linear and angular velocities
        move_cmd.linear.x = 2.0  # Linear velocity
        move_cmd.angular.z = 1.0  # Angular velocity

        rospy.loginfo("Moving the turtle in a circle...")

        # Loop until the node is shutdown
        while not rospy.is_shutdown():
            # Publish the move command to the turtle
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

        # Stop the turtle when done
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.loginfo("Stopping the turtle.")
        # Stop the turtle by sending an empty Twist message
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        CircleMover()
    except rospy.ROSInterruptException:
        rospy.loginfo("Circle movement node terminated.")
