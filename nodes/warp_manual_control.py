#!/usr/bin/env python
# Copyright (c) 2012, Falkor Systems, Inc.  All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  THIS SOFTWARE IS
# PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
# Because the package ardrone_autonomy has not yet been catkinized and roslib.load manifest won't work..use this hack!
# You may change the path to your ardrone_autonomy folder. This is needed to actually import the navdata messages, you
# can also append your $PYTHONPATH
sys.path.append("/opt/ros/groovy/stacks/ardrone_autonomy/src")
import rospy
import std_srvs.srv
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy, Image
from ardrone_autonomy.msg import Navdata

# TODO: Hover Mode, Flat Trim Request via Button Press

class warpManualControl:
    def __init__(self):
        """
        TODO
        """
        print ">> Waiting for Autonomy Driver to come up."
        rospy.wait_for_service("ardrone/setledanimation")
        print ">> Driver has been started. OK."
        print ">> Now initialising Manual Control."
        self.cmd_vel_pub = rospy.Publisher("goal_vel", Twist)
        # cmd_vel, without a PID controller this is really, really jerky!
        # self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.land_pub = rospy.Publisher("ardrone/land", Empty)
        self.land_pub = rospy.Publisher("ardrone/land", Empty)
        self.takeoff_pub = rospy.Publisher("ardrone/takeoff", Empty)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.callback_joy)
        self.navdata_sub = rospy.Subscriber("ardrone/navdata", Navdata, self.callback_nav)
        self.angularZlimit = 3.141592 / 2
        self.linearXlimit = 2
        self.linearZlimit = 2
        self.battery_status = 0.0

    def greeting(self):
        print ">> Manual Control Initialised"

    def takeoff(self):
        self.takeoff_pub.publish(Empty())

    def land(self):
        self.land_pub.publish(Empty())

    def callback_nav(self, data):
        """
        Callback which is registered in the instantiation of the subscriber navdata_sub.
        :param data: Incoming data from ARAutonomy Drone NavData topic: velocity, height, yaw, pitch, roll, etc.
        """
        if self.battery_status != data.batteryPercent:
            print ">> Battery Status (percent left): %s" % data.batteryPercent
            self.battery_status = data.batteryPercent

    def callback_joy(self, data):
        """
        Callback which is registered in the instantiation of the subscriber joy_sub.
        This setup is optimized for the MicroSoft XBox controller. You might need to adjust the button
        indexes to match your product.
        :param data: Incoming data from the Joy node topic: axes and buttons of your gamepad/joystick
        """
        if data.buttons[0] == 1:
            self.takeoff()

        if data.buttons[1] == 1:
            self.land()

        # Do cmd_vel
        self.current_cmd = Twist()
        self.current_cmd.angular.x = self.current_cmd.angular.y = 0

        # Forwards
        # Smooth it a little, round(n, amount)
        self.current_cmd.linear.x = round(data.axes[1], 1) * self.linearXlimit

        # Turn
        self.current_cmd.angular.z = round(data.axes[3], 1) * self.angularZlimit

        # Upwards
        if data.buttons[4] == 1:
            self.current_cmd.linear.z = data.buttons[4] * self.linearZlimit

        # Downwards
        if data.buttons[5] == 1:
            self.current_cmd.linear.z = (data.buttons[5] * self.linearZlimit) * -1

        # Strife left and right
        self.current_cmd.linear.y = data.axes[6] * self.linearXlimit

        # Hover Mode "on"
        # TODO

        # Do cmd_vel
        self.cmd_vel_pub.publish(self.current_cmd)


def main():
    rospy.init_node("warp_manual")
    mc = warpManualControl()
    mc.greeting()
    r = rospy.Rate(50)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main()