#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
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
        x = msg.linear.x
        z = msg.angular.z

        rospy.loginfo("X = {}, Z = {}".format(10*x, 10*z))
        self.joystick.set_percent(10*x,10*z)


if __name__ == "__main__":
    rospy.init_node("wheelchair_virtual_joystick_driver")

    wrapper = Wheelchair_virtual_joystick_driver()
    rospy.on_shutdown(wrapper.stop)

    rospy.loginfo("Joystick driver running. Controller can be turned on")
    rospy.spin()