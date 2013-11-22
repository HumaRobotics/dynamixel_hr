#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

        
import roslib
import rospy
from threading import Thread

from sensor_msgs.msg import *
from std_msgs.msg import *

class DxlROS(Thread):
    def __init__(self,chain,bindings=None):
        Thread.__init__(self)
        self.chain=chain        
        self.bindings=bindings
        
        if self.bindings==None: # Use IDs as names if no binding provided
            b={}
            for id in self.chain.motors.keys():
                b[id]=str(id)
            self.bindings=b
                    
        for (id,name) in self.bindings.items():
            if id not in self.chain.motors.keys():
                raise Exception,"Cannot bind ROS name %s to non-existing motor ID %d"%(name,id)
        rospy.Subscriber("/dxl/enable",Bool,self.enable)
        self.buildPublishers()
        self.start()
    
    def run(self):
        r=rospy.Rate(10)
        while not rospy.is_shutdown():
            for (id,name) in self.bindings.items():
                v=self.chain.get_reg(id,"present_position")
                self.pub[id].publish(v)            
            r.sleep()
        
        
        
    
    def buildPublishers(self):
        self.pub={}
        for (id,name) in self.bindings.items():
            self.pub[id]=rospy.Publisher("/dxl/%s/position"%name,Int16)
            
            
        
    def enable(self,msg):
        self.chain.enable(msg.data)
            
        

        
