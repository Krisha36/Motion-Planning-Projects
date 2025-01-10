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
    def __init__(self,goal):
        rospy.init_node('obstacle_avoidance')
        self.rate = rospy.Rate(10)
        self.current_orientation = Quaternion()
        self.goal_list = goal
        self.goal = self.goal_list[0]
        self.i=0  # Goal location
        print("goal received",self.goal)
        self.goal_reached = False
        self.alpha = 1  # Attraction coefficient
        self.beta = 5.0  # Repulsion coefficient
        self.q_max = 2.5  # Maximum distance of influence for repulsion
        self.max_linear_vel = np.array([0.22 , 0.22]) # Maximum linear velocity
        self.max_angular_vel = 0.1  # Maximum angular velocity
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Loop()
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.get_pose)
        self.Loop()


    def Loop(self):
        while not rospy.is_shutdown():
            if not self.goal_reached:

                net_force = self.calculate_force()
                net_force = np.matmul(self.transformation_matrix,net_force)
                print(net_force,'force')

                # Convert force to velocity commands
                linear_vel,angular_vel = self.convert_to_velocity(net_force)

                linear_vel = np.minimum(linear_vel, self.max_linear_vel)
                linear_vel = np.maximum(linear_vel,(-1)*self.max_linear_vel)
                angular_vel = min(angular_vel, self.max_angular_vel)
                angular_vel = max(angular_vel,(-1)*self.max_angular_vel)

                # Publish velocity commands
                self.publish_velocity(linear_vel,angular_vel)
                #print(self.current_position)
                #print(np.linalg.norm(self.goal - self.current_position))
                # Check if goal is reached
                if np.linalg.norm(self.goal - self.current_position) < 0.2:
                    self.i = self.i +1
                    self.goal_reached = True

            else: 
                self.myhook()


            


    def get_pose(self,data):
        print('hi')
        data = data.pose.pose
        self.positionx = data.position.x
        self.positiony = data.position.y
        self.current_position = np.array([self.positionx,self.positiony])
        print(self.current_position)
        #self.current_position.z = data.position.z
        self.current_orientation.x = data.orientation.x
        self.current_orientation.y = data.orientation.y
        self.current_orientation.w = data.orientation.w
        self.current_orientation.z = data.orientation.z
        self.current_pitch, self.current_roll, self.yaw = self.get_euler_angles()
        self.compute_transformation_matrix()

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
        #print(yaw,'yaw')
        return roll, pitch, yaw

    def compute_transformation_matrix(self):
        self.transformation_matrix = np.array([[math.cos(self.yaw), math.sin(self.yaw)],[-math.sin(self.yaw), math.cos(self.yaw)]])


    #def odom_callback(self,msg):
    #    self.positionx = msg.pose.pose.position.x
    #    self.positiony = msg.pose.pose.position.y
    #    self.bot_angle = math.atan2(self.positiony,self.positionx)
    #    self.current_position = np.array([self.positionx,self.positiony])
    #    print(self.current_position,'curr pos')

    def laser_callback(self, scan_msg):
    
        self.ranges = scan_msg.ranges
        
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max



    

    def myhook(self):
        self.goal_reached = False
        if(self.i>2):
            rospy.on_shutdown()
        else:
            self.goal = self.goal_list[self.i]
            print("goal chnged", self.goal)

    def calculate_force(self):
        force = np.array([0,0])
        force_rep=0
        force_att = self.attraction_force()
        #print(force_att,'att')
        for i in range(len(self.ranges)):
        #i = np.argmin(self.ranges)
            if self.ranges[i] != np.inf:
                ray_angle = self.angle_min + (i * self.angle_increment)
                angle = self.yaw + ray_angle
                force_rep_mag = self.repulsion_force(self.ranges[i])
                force_rep += np.array([math.sin(angle)*force_rep_mag,math.cos(angle)*force_rep_mag])
        print(force_rep,'rep')
        force = force_att+force_rep



        #for angle, distance in enumerate(scan_msg.ranges):
        #    if distance > 0.1:  # Ignore invalid readings
        #        # Calculate force from obstacle
        #        force_magnitude = self.attraction_force()
        #        print(force_magnitude,'att')
        #        force_rep = self.repulsion_force(distance)
        #        force_magnitude+=self.repulsion_force(distance)
        #        print(force_magnitude,'total')
        #        #force_direction = np.deg2rad(angle)  # Convert angle to radians
        #        #force = force_magnitude * np.array([np.cos(force_direction), np.sin(force_direction)])
        #        forces.append(force_magnitude)
#
        ## Calculate net force from all obstacles
        #net_force = np.sum(forces, axis=0)

        return force

    def dist(self,p,q):
        return math.sqrt(((p[0]-q[0])**2 )+((p[1]-q[1])**2 ))

    def attraction_force(self):
        if self.dist(self.current_position,self.goal) < 2:
            return self.alpha*(self.current_position-self.goal)
        else:
            return self.alpha*(self.current_position-self.goal)*2/self.dist(self.current_position,self.goal)

    def repulsion_force(self, distance):
        #print(distance,'dist')
        if distance < self.q_max:
            #print('if')
            return self.beta * ((1 / self.q_max) - (1 / distance)) * ((1 / distance) ** 2)
        else:
            #print('else')
            return 0

    def convert_to_velocity(self, force):
        # Convert force to linear and angular velocity commands
        #print(force,'hi')
        linear_vel = (-1)*force
        linear_vel_mag = np.linalg.norm(linear_vel)
        angular_vel = math.atan2(linear_vel[1],linear_vel[0])/10
        linear_vel = linear_vel/10 # Scale linear velocity
        #angular_vel = np.arctan2(force[1], force[0])  # Calculate angular velocity
        return linear_vel,angular_vel

    def publish_velocity(self, linear_vel,angular_vel):
        # Publish linear and angular velocity commands
        vel_cmd = Twist()
        #print(abs(linear_vel[0]))
        #if abs(linear_vel[0]) > 0.05 :
        #    vel_cmd.linear.x = linear_vel[0]
        #    vel_cmd.angular.z = 0
        #else:
        #    vel_cmd.angular.z = 0.3
        #    vel_cmd.linear.x = 0
        #print(linear_vel, 'ff')
        rospy.sleep(1)
        vel_cmd.linear.x = linear_vel[0]
        #new_direction = math.atan2(linear_vel[1], linear_vel[0])
        #delta_angle = new_direction - self.bot_angle
        #print(delta_angle)
        #vel_cmd.angular.z = np.minimum(delta_angle/30,self.max_angular_vel)
        vel_cmd.angular.z = angular_vel
        #print(vel_cmd)
        self.velocity_publisher.publish(vel_cmd)

    def stop(self):
        # Stop the robot
        stop_cmd = Twist()
        self.velocity_publisher.publish(stop_cmd)

if __name__ == '__main__':
    try:
        
        goal = [[2,2],[-1,-1],[1,2]]
        obstacle_avoidance = ObstacleAvoidance(goal)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
