#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class PositionBasedFigureEight:
    def __init__(self):
        rospy.init_node('position_based_figure_eight', anonymous=False)

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.turtle_pose = Pose()
        self.reached_center = False  

        self.position_threshold = 0.05  
        self.center_x = 5.5 
        self.center_y = 5.5  
        self.move_cmd = Twist()
        try:
            self.move_cmd.linear.x = self.get_user_input("Enter a linear velocity (2.0 to 6.0): ", 2.0, 6.0)
            self.move_cmd.angular.z = self.get_user_input("Enter an angular velocity (1.0 to 3.0): ", 1.0, 3.0)
        except EOFError:
            self.move_cmd.linear.x = 3.0
            self.move_cmd.angular.z = 1.5
            rospy.loginfo(f"Using default values: linear velocity = {self.move_cmd.linear.x}, "
                          f"angular velocity = {self.move_cmd.angular.z}")

        rospy.loginfo(f"Starting figure-eight movement using linear velocity: {self.move_cmd.linear.x} "
                      f"and angular velocity: {self.move_cmd.angular.z}")

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

        if self.is_near_center():
            if not self.reached_center:
                self.switch_direction()
                self.reached_center = True  
        else:
            self.reached_center = False

    def is_near_center(self):
        """ Check if the turtle is near the center of the screen. """
        return abs(self.turtle_pose.x - self.center_x) < self.position_threshold and \
               abs(self.turtle_pose.y - self.center_y) < self.position_threshold

    def switch_direction(self):
        """ Switch the direction of the turtle's angular velocity to form the figure-eight pattern. """
        rospy.loginfo(f"Switching direction at position: x={self.turtle_pose.x}, y={self.turtle_pose.y}")

        self.move_cmd.angular.z = -self.move_cmd.angular.z

    def keep_moving(self):
        """ Keep publishing the move command indefinitely. """
        rate = rospy.Rate(100)  
        while not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the turtle.")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        PositionBasedFigureEight()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position-based figure-eight movement node terminated.")