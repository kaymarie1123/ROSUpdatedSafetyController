#!/usr/bin/python
#

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyControllerNode:
    def __init__(self): 
	# subscribe to incoming laser scan data
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
    	# subscribe to incoming Ackermann drive commands
        rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)

        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10) 
    	self.DRIVE = True
        self.DANGER_DISTANCE = .25
        self.ANGLE = 40*np.pi/180
	
    def laser_callback(self,msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges
	
        for index in range(0,len(ranges)):
            angle = angle_min + angle_inc*index

            LOW = -self.ANGLE/2
            HIGH = self.ANGLE/2
	
            if angle >LOW and angle < HIGH:
                if ranges[index] < self.DANGER_DISTANCE:
                    self.DRIVE = False
                    self.ackermann_cmd_input_callback(AckermannDriveStamped())
                    break

    def ackermann_cmd_input_callback(self, msg):
        # republish the input as output (not exactly "safe")
        print self.DRIVE
        if self.DRIVE:
            self.cmd_pub.publish(msg)
        else:
            stop_msg = AckermannDRiveStamped()
            stop_msg.drive.speed = 0
            stop_msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(stop_msg)


if __name__ == "__main__":
    rospy.init_node("safety_controller")
    node = SafetyControllerNode()
    rospy.spin()
