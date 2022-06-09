#!/usr/bin/env python
# from sklearn.utils import column_or_1d
import rospy
import math
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
# import numpy as np
from std_msgs.msg import Bool

# def talk_to_me():
#     # rospy.init_node('publisher_node', anonymous=True)
#     rate=rospy.Rate(1)
#     rospy.loginfo("Publisher Node Started, now publishing messages")
#     while not rospy.is_shutdown():
#         print("Namaste")
#         # print(cloud)
#         rate.sleep()
class ObjectDetector():
    def __init__(self):
        self.pub = rospy.Publisher("/findcoordinates", Point , queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.spin()
# def totuple(a):
#     try:
#         return tuple(totuple(i) for i in a)
#     except TypeError:
#         return a

    def callback(self, data):
        # self.i=0
        self.cloud=data

        # for _ in range(len(cloud.ranges)):
        #     cloud.ranges[0] = 0
        # cloud.ranges[0:30] = 0
        #array1 = np.asarray(cloud.ranges)
        #array1[range(len(cloud.ranges))] = 0
        #array1[range(20)] = 20
        #cloud.ranges = totuple(array1)
        # print(array1)
        # print(cloud.ranges)
        # if(self.cloud):
        #     i+=1
        #self.d=5
        self.f(self.cloud)
        #self.findcoordinate(self.i)

    def f(self, cloud):
        point = Point()

#        self.regions =min(min(cloud.ranges[0:1146]),10000)
        self.regions =min(min(cloud.ranges[518:624]),10000)
        self.minm=0
        self.i=0
        while cloud.ranges[self.minm]!=self.regions:
            self.i+=1
            self.minm+=1
            # return regions
        self.theta=(360/1147)*self.i
        # if(theta<90):
        #     ansx=math.cos(math.radians(theta))
        #     ansy=-(math.sin(math.radians(theta)))
        # elif(theta<180):
        #     ansx=math.cos(math.radians(theta))
        #     ansy=(math.sin(math.radians(theta)))
        # elif(theta<270):
        #     ansx=-(math.cos(math.radians(theta)))
        #     ansy=(math.sin(math.radians(theta)))
        # else:
        rospy.loginfo(self.regions)
        print(self.theta)
        ansx= (self.regions)*((math.cos(math.radians(self.theta))))
        ansy= (self.regions)*((math.sin(math.radians(self.theta))))
        point.x = ansx
        point.y = ansy
        point.z = 0
        #print("fds")
        self.talk(point)
     
    def talk(self, ans):
        # rate = rospy.Rate(1)
        #while not rospy.is_shutdown():

        self.pub.publish(ans)
            # rate.sleep()
    # ans=f(cloud)
    # pub.publish(ans)



# cloud = PointCloud2()

if __name__=='__main__':
    rospy.init_node('findcoordinate', anonymous=True)
    ObjectDetector()

