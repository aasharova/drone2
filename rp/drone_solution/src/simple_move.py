#!/usr/bin/env python

import time

import rospy
from geometry_msgs.msg import Twist

from hector_uav_msgs.srv import EnableMotors


class SimpleMover():

    def __init__(self):
        rospy.init_node('simple_mover', anonymous=True)
        self.rate = rospy.Rate(30)

        # (1) Initialize publisher to 'cmd_vel' topic
        # # TODO Uncomment and Modify following code to 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.motor = rospy.ServiceProxy('/enable_motors', EnableMotors)
        
        #rospy.spin()

        rospy.on_shutdown(self.shutdown)

    def enable_motors(self):
        # (2) Call here the ros-service 'enable_motors')
        # TODO write code here
        rospy.wait_for_service('/enable_motors')
        try:
            # enable_motors = EnableMotors()
            # enable_motors.enable = True
            self.motor(True)
        except:
            pass
            

    def take_off(self):
        self.enable_motors()
        start_time = time.time()
        end_time = start_time + 3

        # (3) Set the linear velocity by z axis during 3 seconds
        # TODO write code here
        while time.time < end_time:
            cmd_vel = Twist()
            cmd_vel.linear.z = 5.0
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.loginfo(cmd_vel.linear.z)

        #z = msg.pose.pose.position.z
        # dz = msg.twist.twist.linear.z

        # error = 5 - z
        # derror = 0.0 - dz

        # kp = 0.1
        # kd = 0.0



        # u_msg = Twist()
        # u_msg.linear.z = kp + error + kd*derror

        # pub_cmd.publish(u_msg)

    def spin(self):

        self.take_off()

        while not rospy.is_shutdown():

            # (4) Set the linera velocity by x axis is equal to 0.5 m/sec
            # TODO write code here
            twist_msg = Twist()
            twist_msg.linear.z = 5.0
            self.cmd_vel_pub.publish(twist_msg)

            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__=="__main__":
    simple_mover = SimpleMover()
    simple_mover.spin()
