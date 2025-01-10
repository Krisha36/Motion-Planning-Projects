#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Quaternion
import numpy as np
from nav_msgs.msg import Odometry
import math
import time

class ObstacleAvoidance:

    def __init__(self,goalx,goaly):

        rospy.init_node('obstacle_avoidance')
        self.rate = rospy.Rate(10)
        self.current_orientation = Quaternion()
        #self.goal_list = goal
        self.goal = [int(goalx),int(goaly)]
        print("goal received", self.goal)
        self.goal_reached = False
        self.alpha = 10  # Attraction coefficient
        self.beta = 4 # Repulsion coefficient
        self.q_max = 0.25  # Maximum distance of influence for repulsion
        self.max_linear_vel = 0.22 # Maximum linear velocity
        self.max_angular_vel = 2  # Maximum angular velocity
        self.velocity_publisher = rospy.Publisher('/tb3_7/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/tb3_7/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/tb3_7/odom', Odometry, self.get_pose)
        self.recieved_data = False

    def get_pose(self,data):
        
        data = data.pose.pose
        self.positionx = data.position.x
        self.positiony = data.position.y
        self.current_position = np.array([self.positionx,self.positiony])
        #print(self.current_position, "hi")
        self.current_orientation.x = data.orientation.x
        self.current_orientation.y = data.orientation.y
        self.current_orientation.w = data.orientation.w
        self.current_orientation.z = data.orientation.z
        self.current_pitch, self.current_roll, self.yaw = self.get_euler_angles()

    def get_euler_angles(self):

        x, y, z, w = self.current_orientation.x , self.current_orientation.y, self.current_orientation.z, self.current_orientation.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw


    def loop(self):
        

        if not self.recieved_data:
            pass
        #elif not self.goal_reached:
        force_att = self.attraction_force()
        force = self.force_rep + force_att
        print(self.force_rep,'rep')
        print(force_att,'att')
        print(force,'force')

        force_mag = force[0]*math.cos(self.yaw)+(force[1]*math.sin(self.yaw)/10)
        force_angle = math.atan2(force[1],force[0])
        
        linear_vel = 0

        if force_angle - self.yaw > math.pi:
            force_angle -= 2*math.pi
        elif force_angle - self.yaw < -math.pi:
            force_angle += 2*math.pi
        #print(abs(force_angle - self.yaw),'prob', math.pi/2)
        if (abs(force_angle - self.yaw) > math.pi/2):
            if (force_angle - self.yaw > 0):
                force_angle -= math.pi
                
            else:
                force_angle += math.pi
                
            linear_vel = abs(force_mag)/9

        else:
            
            linear_vel = -abs(force_mag)/9
        #linear_vel = (-1)*force
        #print(linear_vel,'why')
        angular_vel = (force_angle-self.yaw)*3
        linear_vel = min(linear_vel, self.max_linear_vel)
        self.linear_vel = max(linear_vel,(-1)*self.max_linear_vel)
        angular_vel = min(angular_vel, self.max_angular_vel)
        self.angular_vel = max(angular_vel,(-1)*self.max_angular_vel)


        # Check if goal is reached
        if np.linalg.norm(self.goal - self.current_position) < 0.2:
            
            self.goal_reached = True
        #else: 
        #    self.myhook()

    def laser_callback(self, scan_msg):

        self.ranges = scan_msg.ranges
        force_rep = 0
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
        self.recieved_data = True
    
        for i in range(len(self.ranges)-1):
        #i = np.argmin(self.ranges)
            if self.ranges[i] != "inf" and self.ranges[i] != 0.0:
                ray_angle = self.angle_min + (i * self.angle_increment)
                angle = self.yaw + ray_angle
                force_rep_mag = self.repulsion_force(self.ranges[i])
                force_rep += np.array([-force_rep_mag*math.cos(angle) + force_rep_mag*math.sin(angle) , -force_rep_mag*math.sin(angle)+force_rep_mag*math.cos(angle)])
        #print(force_rep,'rep')
        
        self.force_rep = force_rep

    def dist(self,p,q):
        return math.sqrt(((p[0]-q[0])**2 )+((p[1]-q[1])**2 ))

    def attraction_force(self):

        if self.dist(self.current_position,self.goal) < 3:
            return self.alpha*(self.current_position-self.goal)
        else:
            return self.alpha*(self.current_position-self.goal)*3/self.dist(self.current_position,self.goal)

    def repulsion_force(self, distance):

        #print(distance,'dist')
        if distance < self.q_max:
            return self.beta * ((1 / self.q_max) - (1 / distance)) * ((1 / distance) ** 2)
        else:
            return 0

    

    def publish_velocity(self):

        # Publish linear and angular velocity commands
        vel_cmd = Twist()
        vel_cmd.linear.x = self.linear_vel
        vel_cmd.angular.z = self.angular_vel
        print(vel_cmd)
        
        return vel_cmd

    def run(self):

        while not rospy.is_shutdown():
            self.loop()  # Call the obstacle avoidance function
            self.velocity_publisher.publish(self.publish_velocity())
            self.rate.sleep()
        

if __name__ == '__main__':
    try:
        arr = sys.argv[1].split(',')
        goalx = arr[0]
        goaly = arr[1]
        obstacle_avoidance = ObstacleAvoidance(goalx,goaly)
        rospy.sleep(10)
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
