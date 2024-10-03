#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class SwimToGoal:
    def __init__(self):
        rospy.init_node('swim_to_goal', anonymous=False)

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.turtle_pose = Pose()
        self.rate = rospy.Rate(10)  

        self.V_gain = 1.0  
        self.A_gain = 4.0  

        self.goal_reached = False

        self.goal_x = float(input("Set your x goal: "))
        self.goal_y = float(input("Set your y goal: "))
        self.tolerance = float(input("Set your tolerance: "))

        self.swim_to_goal()

    def update_pose(self, data):
        self.turtle_pose = data

    def euclidean_distance(self, goal_x, goal_y):
        return math.sqrt(pow((goal_x - self.turtle_pose.x), 2) + pow((goal_y - self.turtle_pose.y), 2))

    def linear_velocity(self, goal_x, goal_y):
        return self.V_gain * self.euclidean_distance(goal_x, goal_y)

    def steering_angle(self, goal_x, goal_y):
        return math.atan2(goal_y - self.turtle_pose.y, goal_x - self.turtle_pose.x)

    def angular_velocity(self, goal_x, goal_y):
        return self.A_gain * (self.steering_angle(goal_x, goal_y) - self.turtle_pose.theta)

    def swim_to_goal(self):
        move_cmd = Twist()

        while not rospy.is_shutdown() and not self.goal_reached:
            distance = self.euclidean_distance(self.goal_x, self.goal_y)

            if distance < self.tolerance:
                rospy.loginfo("Goal reached!")
                self.stop_turtle()
                self.goal_reached = True
                break

            move_cmd.linear.x = self.linear_velocity(self.goal_x, self.goal_y)
            move_cmd.angular.z = self.angular_velocity(self.goal_x, self.goal_y)

            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

    def stop_turtle(self):
        rospy.loginfo("Stopping the turtle.")
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)

if __name__ == '__main__':
    try:
        SwimToGoal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
