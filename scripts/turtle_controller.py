#!/usr/bin/env python



import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from PID import PID
import math


class turtle_PID():

    def __init__(self):

        self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)

        rospy.init_node('turtle_controller', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz

        self.angle_PID = PID()
        self.distance_PID = PID()

        self.angle_PID.setKp(1.4)
        self.angle_PID.setKi(0)
        self.angle_PID.setKd(0)

        self.distance_PID.setKp(1.4)
        self.distance_PID.setKi(0)
        self.distance_PID.setKd(0)

        self.msg = Twist()

        self.move_turtle()


    def angular_controller(self):

        self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))

        self.xr = self.R*math.cos(self.current_angle)
        self.yr = self.R*math.sin(self.current_angle)

        self.xim = self.current_pose_x + self.xr
        self.yim = self.current_pose_y + self.yr

        self.C = math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))

        if self.xim > self.goal_x:

            self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        else:
            self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        
        print self.alpha
        while self.alpha>0.005: 
            self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))
            #print "dentro do while"
            self.xr = self.R*math.cos(self.current_angle)
            self.yr = self.R*math.sin(self.current_angle)

            self.xim = self.current_pose_x + self.xr
            self.yim = self.current_pose_y + self.yr

            self.C = math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))
            
            if self.xim > self.goal_x:

                self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
            
            else:
                
                self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

            self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

            self.PID_angle = self.angle_PID.update(self.alpha)

            self.msg.angular.z = self.PID_angle

            self.pub.publish(self.msg)

    

    def distance_controller(self):

        self.distance = math.sqrt(math.pow(self.goal_x - self.current_pose_x , 2) + math.pow(self.goal_y - self.current_pose_y, 2 ))
        #self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))
        print "distance: " + str(self.distance)
        while self.distance > 0.15:

            self.distance = math.sqrt(math.pow(self.goal_x - self.current_pose_x , 2) + math.pow(self.goal_y - self.current_pose_y, 2 ))

            self.PID_distance = self.distance_PID.update(self.distance)

            self.msg.linear.x = self.PID_distance

            self.pub.publish(self.msg)

            

    def get_user_input(self):

        self.goal_x = input("X position:")
        self.goal_y = input("Y position:")

    def move_turtle(self):

        self.get_user_input()

        self.angular_controller()

        self.distance_controller()

     

    def pose_callback(self,data):

        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta

if __name__ == '__main__':

    try:

        turtle_PID()

    except rospy.ROSInterruptException:

        pass