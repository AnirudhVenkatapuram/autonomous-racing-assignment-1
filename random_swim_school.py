#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
import random  

class RandomPositionFigureEight:
    def __init__(self):
        rospy.init_node('random_position_figure_eight', anonymous=False)

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.turtle_pose = Pose()
        self.reached_start = False  

        self.position_threshold = 0.1  

        self.start_x = random.uniform(3.0, 8.0)
        self.start_y = random.uniform(3.0, 8.0)

        self.move_cmd = Twist()
        self.move_cmd.linear.x = random.uniform(2.0, 6.0)  
        self.move_cmd.angular.z = random.uniform(1.0, 3.0)  

        self.last_teleport_time = rospy.Time.now()

        self.teleport_to_position(self.start_x, self.start_y, pen_off=True)

        rospy.loginfo(f"Starting figure-eight pattern from random position: x={self.start_x}, y={self.start_y}")
        rospy.loginfo(f"Random linear velocity: {self.move_cmd.linear.x}, Random angular velocity: {self.move_cmd.angular.z}")

        self.keep_moving()

    def set_pen_state(self, r, g, b, width, off):
        """ Set the pen state of the turtle. """
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            set_pen(r, g, b, width, off)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set pen state: {e}")

    def pose_callback(self, data):
        """ Callback function to get the current position of the turtle. """
        self.turtle_pose = data

        if self.is_near_start():
            current_time = rospy.Time.now()
            if current_time - self.last_teleport_time > rospy.Duration(0.25):  
                self.last_teleport_time = current_time  
                self.move_cmd.angular.z = -self.move_cmd.angular.z
                self.teleport_to_start()  

    def is_near_start(self):
        """ Check if the turtle is near the start position of the circle. """
        return abs(self.turtle_pose.x - self.start_x) < self.position_threshold and \
               abs(self.turtle_pose.y - self.start_y) < self.position_threshold

    def teleport_to_start(self):
        """ Teleport the turtle back to the starting position with the pen turned on. """
        self.teleport_to_position(self.start_x, self.start_y, pen_off=False)

    def teleport_to_position(self, x, y, pen_off):
        """ Teleport the turtle to a specified position (x, y). If pen_off is True, turn the pen off during teleportation. """
        if pen_off:
            self.set_pen_state(255, 255, 255, 1, 1) 

        rospy.wait_for_service('/turtle1/teleport_absolute')
        try:
            teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
            teleport_turtle(x, y, 0.0)  

            if pen_off:
                self.set_pen_state(0, 0, 0, 2, 0) 
                rospy.loginfo("Teleported to initial position with pen turned off, pen turned back on now.")
            else:
                rospy.loginfo("Teleported to position with pen on (drawing).")
        except rospy.ServiceException as e:
            rospy.logerr(f"Teleportation failed: {e}")

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
        RandomPositionFigureEight()
    except rospy.ROSInterruptException:
        rospy.loginfo("Random position figure-eight movement node terminated.")
