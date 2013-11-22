#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

        
import logging
import roslib
import rospy
from threading import Thread

from sensor_msgs.msg import *
from std_msgs.msg import *

class DxlROS(Thread):
    def __init__(self,chain,rate=10,bindings=None,raw=True):
        Thread.__init__(self)
        self.chain=chain
        self.rate=rate
        self.bindings=bindings
        self.raw=raw
        
        if self.bindings==None: # Use IDs as names if no binding provided
            b={}
            for id in self.chain.motors.keys():
                b[id]=str(id)
            self.bindings=b
                    
        for (id,name) in self.bindings.items():
            if id not in self.chain.motors.keys():
                raise Exception,"Cannot bind ROS name %s to non-existing motor ID %d"%(name,id)

        logging.info("Creating ROS elements")
        
        rospy.Subscriber("/dxl/enable",Bool,self.enable)
        
        self.buildPublishers()
        self.buildSubscribers()
        
        self.start()
    
    def run(self):
        r=rospy.Rate(float(self.rate))
        while not rospy.is_shutdown():
            if self.raw:
                for id,motorname in self.bindings.items():
                    regs=self.chain.motors[id].registers
                    basename="/dxl/%s/"%motorname
                    for regname,reg in regs.items():
                        topic=basename+regname
                        v=self.chain.get_reg(id,regname)
                        self.pub[topic].publish(v)            
            r.sleep()
        
        
        
    
    def buildPublishers(self):
        self.pub={}
        print("Creating ROS Publishers")
        if self.raw:
            for id,motorname in self.bindings.items():            
                regs=self.chain.motors[id].registers
                basename="/dxl/%s/"%motorname
                for regname,reg in regs.items():
                    topic=basename+regname
                    print("Creating ROS Publisher: %s"%topic)
                    self.pub[topic]=rospy.Publisher(topic,Int32)

    def buildSubscribers(self):
        if self.raw:
            for id,motorname in self.bindings.items():            
                regs=self.chain.motors[id].registers
                basename="/dxl/%s/"%motorname
                for regname,reg in regs.items():
                    if 'w' in reg.mode:
                        topic=basename+regname+"/set"
                        print("Creating ROS Subscriber: %s"%topic)
                        rospy.Subscriber(topic,Int32,lambda msg,id=id,regname=regname: self.set_register(msg,id,regname) )
            
            
            
        
    def enable(self,msg):
        self.chain.enable(msg.data)
            
    def set_register(self,msg,id,regname):
        v=msg.data
        print "ros call ID %d reg %s val %d"%(id,regname,v)
        self.chain.set_reg(id,regname,v)

        
