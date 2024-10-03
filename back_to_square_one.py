#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative
from std_srvs.srv import Empty

class SquareDrawer:
    def __init__(self):
        rospy.init_node('back_to_square_one', anonymous=False)

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.wait_for_service('/clear')
        rospy.wait_for_service('/turtle1/set_pen')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        rospy.wait_for_service('/turtle1/teleport_relative')

        self.clear_bg = rospy.ServiceProxy('/clear', Empty)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.teleport_relative = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)

        self.side_length = self.get_user_input("Enter the length of the side of the square (1 to 5): ", 1.0, 5.0)

        self.change_background_color_to_red()

        self.teleport_to_start_position()

        self.set_pen_state(0, 0, 0, 3, 0)  

        self.draw_square(self.side_length)

        self.stop_turtle()

    def get_user_input(self, prompt, min_val, max_val):
        while True:
            try:
                value = float(input(prompt))
                if min_val <= value <= max_val:
                    return value
                else:
                    print(f"Please enter a value between {min_val} and {max_val}.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")

    def change_background_color_to_red(self):
        rospy.loginfo("Changing the background color to red using ROS parameters...")

        rospy.set_param('/turtlesim/background_r', 255)  
        rospy.set_param('/turtlesim/background_g', 0)    
        rospy.set_param('/turtlesim/background_b', 0)    

        try:
            self.clear_bg()
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear screen for background color change: {e}")

    def teleport_to_start_position(self):
        rospy.loginfo("Teleporting turtle to start position (1, 1) without drawing lines...")

        self.set_pen_state(255, 255, 255, 5, 1)  

        try:
            self.teleport_turtle(1.0, 1.0, 0.0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to teleport turtle: {e}")

        self.set_pen_state(0, 0, 0, 3, 0) 

    def set_pen_state(self, r, g, b, width, off):
        """ Set the pen state to control drawing lines.
            r, g, b: Color of the pen.
            width: Width of the pen line.
            off: 1 to disable the pen, 0 to enable. """
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            set_pen(r, g, b, width, off)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call set_pen service: {e}")

    def draw_square(self, side_length):
        """ Draw a square with the given side length. """
        move_cmd = Twist()

        linear_speed = 1.0  
        move_cmd.linear.x = linear_speed

        side_time = side_length / linear_speed

        rate = rospy.Rate(10) 

        for _ in range(4):  
            move_cmd.linear.x = linear_speed
            move_cmd.angular.z = 0.0
            rospy.loginfo(f"Drawing side with length {side_length}")
            t0 = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - t0) < side_time:
                self.cmd_vel.publish(move_cmd)
                rate.sleep()

            move_cmd.linear.x = 0.0
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(0.1)  

            try:
                self.teleport_relative(0, 1.5708) 
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to teleport turtle for turning: {e}")
            rospy.sleep(0.1)  
    def stop_turtle(self):
        rospy.loginfo("Stopping the turtle...")
        stop_cmd = Twist()  
        self.cmd_vel.publish(stop_cmd)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        SquareDrawer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Square drawing node terminated.")
