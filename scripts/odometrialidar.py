#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time
import math
import icp_example

class odometrialidar:
    def __init__(self):
        rospy.init_node("odomlidar", anonymous=False)
        rospy.Subscriber('/mestre/scan', LaserScan,self.update_lidar)
        self.rate=rospy.Rate(20)
        self.scan=LaserScan() #mensagem dos dados do lidar
        self.ponts1=[[],[]]
        self.ponts2=[[],[]]
        self.x=0
        self.y=0
        self.yaw=0
        self.controle=0
        self.controleHZ=0

    def update_lidar(self, msg):
        if(self.controleHZ==2):#calcula a odometria a cada segundo para ==5
            print("x: %.2f y: %.2f  ang: %.2f" %(odom.x,odom.y,(odom.yaw*180/math.pi)))
            self.controleHZ=0
            self.scan=msg
            #tam=len(self.scan.ranges)
            data=np.asarray(self.scan.ranges)

            x = []; y = []
            for r,deg in zip(data, enumerate(data)):
                if r != np.inf:
                    x.append( r * np.cos( deg[0] * np.pi/180 ))
                    y.append( r * np.sin( deg[0] * np.pi/180 ))
            #plt.plot(x,y,'.')#teste
            #plt.plot(x,y,'.')
            T=[]
            R=[[],[]]

            if(self.controle==0):
                self.ponts1=np.vstack((x,y))
                self.controle=1
                return
            elif(self.controle==1):
                self.ponts2=np.vstack((x,y))
                self.controle=2
                R, T=self.calcOdom(self.ponts1,self.ponts2)
            else:
                self.ponts1=np.vstack((x,y))
                R, T=self.calcOdom(self.ponts2,self.ponts1)
                self.controle=1
            if(abs(T[0])>0.02):
                self.x+=T[0]
            if(abs(T[1])>0.02):
                self.y+=T[1]
            if(abs(R[0,1])>0.02):    
                self.yaw+=R[0,1]
                self.yaw%=(2*math.pi)
        else:
            self.controleHZ+=1

    def calcOdom(self,pk,pk1):
        #dimensiona os vetores pk, e pk1 para as mesmas dimenções
        #apagando os pontos exedentes com alguma metodologia que não está implementada
        # para escolher os pontos que serão apgados
        pk,pk1=self.dimensiona(pk,pk1)
        #pk1 <--> p[k+1] pontos atuais

        #icp simples 
        R, T = icp_example.icp_matching(pk, pk1)
        return R, T

    def dimensiona(self, pk,pk1):
        if(len(pk[0,:])>len(pk1[0,:])):
            pk=np.vstack((pk[0,0:len(pk1[0,:])], pk[1,0:len(pk1[0,:])]))
        elif(len(pk1[0,:])>len(pk[0,:])):
            pk1=np.vstack((pk1[0,0:len(pk[0,:])], pk1[1,0:len(pk[0,:])]))
        return pk, pk1


if __name__ == '__main__':
    try:
        odom=odometrialidar()
        while not rospy.is_shutdown():
            odom.rate.sleep()
            

    except rospy.ROSInterruptException:
        pass

