#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic


import rospy
import rosbag
import pymongo
import datetime
from pymongo import MongoClient
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_msgs.msg import Int32
from bike_control.msg import controller
from bike_cv.msg import frames


bagimu = rosbag.Bag('test.bag', 'w')
baggps = rosbag.Bag('gps.bag', 'w')
client = MongoClient()
client = MongoClient('localhost', 27017)
db = client['listenerCombined']
collection = db['IMUData']
longitude = 0
latitude = 0
accelerationx = 0
orientationx = 0
rotationx = 0
accelerationy = 0
orientationy = 0
rotationy = 0
accelerationz = 0
orientationz = 0
rotationz = 0
distance = 0
velocity = 0
target_Velocity = 0
target_Angle = 0
speed = 0
imageArray = None


class listenerCombined:
    
    tv = 0
    global target_Velocity
    global speed

    def getAngularVelocity(self):
        return [rotationx, rotationy, rotationz]

    def getOrientation(self):
        return [orientationx, orientationy, orientationz]

    def getAcceleration(self):
        return [accelerationx, accelerationy, accelerationz]

    def getGPS(self):
        return [longitude,latitude]

    def getObstacleCoord(self):
        return [distance]
        
    def getVelocity(self):
        global target_Velocity
        return [target_Velocity]
        
    def getTargets(self):
        global target_Velocity
        global target_Angle
        #print([target_Velocity, target_Angle])
        return [target_Velocity, target_Angle]
        
    def getSpeed(self):
        return speed
        
    def getCameraFeed(self):
        global imageArray
        print('LISTENER', imageArray[:3])
        return imageArray


def callbackIMU(data):
    
    global accelerationx
    global orientationx
    global rotationx
    global accelerationy
    global orientationy
    global rotationy
    global accelerationz
    global orientationz
    global rotationz
    s = listenerCombined()
    post = {"Date": datetime.datetime.utcnow(),
            "Angular velocity": [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z],
            "Orientation": [data.orientation.x, data.orientation.y, data.orientation.z],
            "Linear acceleration": [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]}
    
    collectionIMU = db.collection
    collectionIMU.insert_one(post)

        #rospy.loginfo('Linear acceleration (X,Y,Z): (%f,%f,%f)', data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z)
        #rospy.loginfo('Orientation (X,Y,Z): (%f,%f,%f)', data.orientation.x,data.orientation.y,data.orientation.z)
        #rospy.loginfo('Angular velocity (X,Y,Z): (%f,%f,%f)', data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)
    #print(s.getAngularVelocity())
    #print(s.getOrientation())
    #print(s.getAcceleration())

    accelerationx = data.linear_acceleration.x
    orientationx = data.orientation.x
    rotationx = data.angular_velocity.x
    accelerationy = data.linear_acceleration.y
    orientationy = data.orientation.y
    rotationy = data.angular_velocity.y
    accelerationz = data.linear_acceleration.z
    orientationz = data.orientation.z
    rotationz = data.angular_velocity.z


    bagimu.write("IMUPublisher", data);

def callbackGPS(data):

    global longitude
    global latitude
    s = listenerCombined()
    post = {"Date": datetime.datetime.utcnow(),
            "Longitude": data.longitude,
            "Latitude": data.latitude}
    
    collectionIMU = db.colletion
    collectionIMU.insert_one(post)

        #rospy.loginfo("Longitude: %f, Latitude %f" % (data.longitude, data.latitude))
    print(s.getGPS())
    longitude = data.longitude
    latitude = data.latitude


    baggps.write("fix", data);


def callbackObstacles(data):
    global distance
    s = listenerCombined()
       
    #rospy.loginfo('Obstacle: %s' % data)
    print(s.getObstacleCoord())
    
    distance = data 
    
def callbackVelocity(data):
    global velocity
    s = listenerCombined()
    
    print(s.getVelocity())
    
    velocity = data

def callbackTargets(data):
    global target_Velocity
    global target_Angle
        
    target_Velocity = data.velocity_angle[0]
    target_Angle = data.velocity_angle[1]
    
    tv = target_Velocity
    
    print('V' + str(target_Velocity))
    print('A' + str(target_Angle))


def callbackImages(data):
    global imageArray

    imageArray = data

def listener(listener_name):
    
    # In ROS, nodes are uniquely named. If two nodes with the same# name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print('init node')
    rospy.init_node(listener_name, anonymous=True)
    print('subsc')
    rospy.Subscriber('IMUPublisher', Imu, callbackIMU)
    rospy.Subscriber('fix', NavSatFix, callbackGPS)
    rospy.Subscriber('ObstacleCoordinates', String, callbackObstacles)
    rospy.Subscriber('BikeVelocity', String, callbackVelocity)
    rospy.Subscriber('GUIData', controller, callbackTargets)
    rospy.Subscriber('CameraFeed', String, callbackImages)
    # spin() simply keeps python from exiting until this node is stopped


    #rospy.spin()

if __name__ == '__main__':
    listener()
