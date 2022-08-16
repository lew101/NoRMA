#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import std_msgs.msg
from driver import Joystick

class Wheelchair_virtual_joystick_driver:
    def __init__(self):
        self.joystick = Joystick()

        calib_fb = -1
        calib_lr = -1

        try:
            if rospy.has_param('/wheelchair_calibrate_fb'):
                calib_fb = rospy.get_param('/wheelchair_calibrate_fb')

            if rospy.has_param('/wheelchair_calibrate_lr'):
                calib_lr = rospy.get_param('/wheelchair_calibrate_lr')
        except KeyError:
            calib_fb = -1
            calib_lr = -1

        if (calib_fb == -1 or calib_lr == -1):
            rospy.loginfo("No calibration values provided. Running calibration now...")
            self.joystick.calibrate()
            
        else:
            self.joystick.set_calibration_values(calib_fb, calib_lr)
        rospy.loginfo("Calibration complete")
        
        self.stop()

        rospy.Subscriber('/cmd_vel', Twist, self.on_twist)
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)

    def stop(self):
        """Called when the wrapper is stopping.
        Will set virtual joystick to 0,0 to stop the motors.
        """
        self.joystick.set_percent(0,0)

    def on_twist(self, msg):
        """Callback for the ROS twist subscriber.

        Args:
            msg (geometry_msgs/Twist): ROS Twist message.
        """
        scale = 10
        x = (msg.linear.x) * scale
        z = (msg.angular.z) * scale

        if (x > 100):
            rospy.loginfo("Trimming x to 100")
            x = 100
        if (x < -100):
            rospy.loginfo("Trimming x to -100")
            x = -100
        if (z > 100):
            rospy.loginfo("Trimming z to 100")
            z = 100
        if (z < -100):
            rospy.loginfo("Trimming z to -100")
            z = -100

        rospy.loginfo("X = {}, Z = {}".format(x, z))
        self.joystick.set_percent(x,z)

    def publish_joy(self):
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            x, z = self.joystick.get_percent()
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            
            joy = Joy()
            joy.header = h
            joy.axes = [x,z]
            joy.buttons = []

            self.joy_pub.publish(joy)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("wheelchair_virtual_joystick_driver")

    wrapper = Wheelchair_virtual_joystick_driver()
    rospy.on_shutdown(wrapper.stop)

    rospy.loginfo("Joystick driver running. Controller can be turned on")

    try:
        wrapper.publish_joy()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()