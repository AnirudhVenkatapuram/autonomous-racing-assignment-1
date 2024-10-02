#!/usr/bin/env python3
# Description: Make the turtle swim in a figure-8 pattern based on user input linear and angular velocities.

import rospy
from geometry_msgs.msg import Twist

class SwimSchool:
    def __init__(self):
        rospy.init_node('swim_school', anonymous=False)
        rospy.loginfo("Starting the Swim School Node")
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rate = rospy.Rate(10)

        linear_velocity = float(input("Enter a linear velocity in the range [2, 6]: "))
        angular_velocity = float(input("Enter an angular velocity in the range [1, 3]: "))

        linear_velocity = max(2, min(6, linear_velocity))
        angular_velocity = max(1, min(3, angular_velocity))

        rospy.loginfo(f"Using linear velocity: {linear_velocity} and angular velocity: {angular_velocity}")

        move_cmd = Twist()
        move_cmd.linear.x = linear_velocity
        move_cmd.angular.z = angular_velocity

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

            move_cmd.angular.z = -angular_velocity
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

            move_cmd.angular.z = angular_velocity

    def shutdown(self):
        rospy.loginfo("Stopping the turtle")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        SwimSchool()
    except rospy.ROSInterruptException:
        rospy.loginfo("Swim School node terminated.")
