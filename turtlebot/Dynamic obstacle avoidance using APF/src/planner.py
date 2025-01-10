#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy.core.numeric import Inf
import matplotlib.pyplot as plt
import math
import numpy as np

class Planner:
    def __init__(self):
        self.l = int(61)
        self.Range = [0]*self.l
        rospy.init_node('dynamic_obstacle_avoidance')
        self.rate = rospy.Rate(10)  # 10 Hz
        self.move_cmd = Twist()
        self.i = 1
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #self.initGraph()

        self.Loop()


        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        

    def laser_callback(self, msg):
        # Retrieve LiDAR data and store it for obstacle avoidance
        #self.scan_data = scan_msg
        n = 61 #number of entries
        self.l = n
        self.Range = [0]*n
        for i in range(int(-(n-1)/2), int((n+1)/2)):
            #print(i)
            if i > 0 or i == 0:
                self.Range[i] = msg.ranges[i]
            else:
                self.Range[i] = msg.ranges[360+i]

    def planner(self):
        # Implement dynamic obstacle avoidance using APF logic here based on LiDAR data
        # For example, let's say we stop if there's an obstacle within 0.5 meters
        if min(self.scan_data.ranges) < 0.5:
            self.move_cmd.linear.x = 0.0
        else:
            self.move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s


    def PlotData(self):
        
    
        self.x = [0]*(self.l)
        self.y = [0]*(self.l)
        
        for i in range(int(-((self.l)-1)/2), int(((self.l)+1)/2)):
            self.x[i] = -self.Range[i]*np.sin(i*np.pi/180)
            self.y[i] = self.Range[i]*np.cos(i*np.pi/180)

            
        self.x[0] = 0 
        

    def initGraph(self):

        self.PlotData()
        self.Forces()

        # You probably won't need this if you're embedding things in a tkinter plot...
        plt.ion()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)
    
        self.ax.set_ylim(30,-30)
        self.ax.set_xlim(-30,30)

        

        
        plt.show()
        
        self.scat1 = self.ax.scatter(self.x, self.y) # Returns a tuple of line objects, thus the comma
        
        self.plotForce = self.ax2.arrow(0, 0, (300 - (self.netFy))/100, self.netFx, width = 0.5)

    def Forces(self):
        self.fx = [0]*(self.l)
        self.fy = [0]*(self.l)
        self.netFx = 0
        self.netFy = 0
        self.size = [0]*(self.l)

        for i in range(int(-((self.l)-1)/2), int(((self.l)+1)/2)):
            
            if self.x[i] != 0:
                #print("true", i)
                self.fx[i] = 1/(self.x[i])
            else:
                #print("Else", self.x[i],"  ", i)
                self.fx[i] = 0

            if self.y[i] != 0:
                self.fy[i] = 1/(self.y[i])
            else:
                self.fy[i] = 0
        
        for l in range(0, len(self.fx)):
            #print(l, ":", self.x[l], "\n")
            self.netFx += self.fx[l]

        for l in range(0, len(self.fy)):
        
            self.netFy += self.fy[l]

        for i in range(int(-((self.l)-1)/2), int(((self.l)+1)/2)):
            self.size[i] = (self.fx[i]**2 + self.fy[i]**2)**0.5
            

    def Motion(self):
        stp = Twist()
        self.velocity_publisher.publish(stp)
        self.move = Twist()
        self.move.linear.x = (100 - (self.netFy))/100
        self.move.angular.z = self.netFx/300

        
        return self.move


    def attraction(position, goal, alpha):
        return alpha(position-goal)

    def repulsion(position, obstacle, beta, q_max):
        return beta*((1/q_max)-(1/dist(position,goal)))(1/dist(position,goal)*dist(position,goal))
            
    def potential_field(grid, goal, alpha, beta, q_max):
        x = []
        y = []
        fx = []
        fy = []
    
        obs_i, obs_j = np.where(grid == 1)
    
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i, j] == 0:
                    
                    # add attraction force
                    force = attraction([i, j], goal, alpha)
    
                    for (oi, oj) in zip(obs_i, obs_j):
                        if np.linalg.norm(np.array([i, j]) - np.array([oi, oj])) < q_max:
                            # add replusion force
                            force += repulsion([i, j], [oi, oj], beta, q_max)
                        
                    x.append(i)
                    y.append(j)
                    fx.append(force[0])
                    fy.append(force[1])
    
        return x, y, fx, fy


    def Loop(self):
        while not rospy.is_shutdown():

            self.PlotData()
            self.Forces()

            #update x and y
            self.ax.cla()
            self.ax.set_ylim(-1,3)
            self.ax.set_xlim(-1,1)
            self.scat1 = self.ax.scatter(self.x, self.y)
            
            self.ax2.cla()
            self.ax2.set_ylim(300,-300)
            self.ax2.set_xlim(-300,300)
            self.plotForce = self.ax2.arrow(0, 0, -self.netFx, ( (self.netFy)), width = 5)
            #print (self.i)
            #plot x and y
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            #self.i += 1

            

            self.velocities = self.Motion()

            self.velocity_publisher.publish(self.velocities)


            print ("Fx",-self.netFx,"\n" , "Fy", -self.netFy)

            rospy.sleep(0.5)


    def run(self):
        while not rospy.is_shutdown():
            self.planner()  # Call the obstacle avoidance function
            self.velocity_publisher.publish(self.move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        Planner = Planner()
        #Planner.run()
    except rospy.ROSInterruptException:
        pass
