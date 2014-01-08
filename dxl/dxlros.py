#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

        
import logging
try:
    import roslib
    import rospy
    from sensor_msgs.msg import *
    from std_msgs.msg import *
except:
    logging.warning("Could not load ROS libraries, make sure ROS is installed and your environment variables are properly set")

from threading import Thread


class DxlROS(Thread):
    default_published=["present_position"]
    default_subscribed=["goal_pos"]
    
    def __init__(self,chain,rate=10,bindings=None,raw=False):
        Thread.__init__(self)
        self.chain=chain
        self.rate=rate
        self.bindings=bindings
        self.raw=raw
        self.publishers=[]
        self.subscribers=[]        
        self.pub={}
        
        self.do_stop=False
        
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
        
        if raw:    
            self.buildRawPublishers()
            self.buildRawSubscribers()
        else:
            self.buildSIPublishers()
            self.buildSISubscribers()

        
        self.start()
    
    def run(self):
        r=rospy.Rate(float(self.rate))
        while not rospy.is_shutdown() and not self.do_stop:
            if self.raw:
                self.publishRaw()
            else:
                self.publishSI()
            r.sleep()
        
    def publishSI(self):
        for id,motorname in self.bindings.items():
            regs=self.chain.motors[id].registers
            basename="/dxl/%s/"%motorname
            for regname in self.default_published:
                topic=basename+regname
                v=self.get_register_si(id,regname)
                self.pub[topic].publish(v)

    
    def publishRaw(self):
        for id,motorname in self.bindings.items():
            regs=self.chain.motors[id].registers
            basename="/dxl/%s/"%motorname
            for regname,reg in regs.items():
                topic=basename+regname
                v=self.chain.get_reg(id,regname)
                self.pub[topic].publish(v) 
        
    
        
        
    def stop(self):
        self.do_stop=True
    
    def buildSIPublishers(self):
        print("Creating SI ROS Publishers")
        for id,motorname in self.bindings.items():
            regs=self.chain.motors[id].registers
            basename="/dxl/%s/"%motorname
            for regname in self.default_published:
                topic=basename+regname
                print("Creating SI ROS Publisher: %s"%topic)
                self.pub[topic]=rospy.Publisher(topic,Float64)                
                self.publishers.append(topic)
                
    
    def buildRawPublishers(self):
        print("Creating ROS Publishers")
        for id,motorname in self.bindings.items():            
            regs=self.chain.motors[id].registers
            basename="/dxl/%s/"%motorname
            for regname,reg in regs.items():
                topic=basename+regname
                print("Creating raw ROS Publisher: %s"%topic)
                self.pub[topic]=rospy.Publisher(topic,Int32)
                self.publishers.append(topic)

    def buildSISubscribers(self):
        for id,motorname in self.bindings.items():
            regs=self.chain.motors[id].registers
            basename="/dxl/%s/"%motorname
            for regname in self.default_subscribed:
                topic=basename+regname+"/set"
                print("Creating SI ROS Subscriber: %s"%topic)
                rospy.Subscriber(topic,Float64,lambda msg,id=id,regname=regname: self.set_register_si(msg,id,regname) )
                self.subscribers.append(topic)
            
    def buildRawSubscribers(self):
        for id,motorname in self.bindings.items():            
            regs=self.chain.motors[id].registers
            basename="/dxl/%s/"%motorname
            for regname,reg in regs.items():
                if 'w' in reg.mode:
                    topic=basename+regname+"/set"
                    print("Creating raw ROS Subscriber: %s"%topic)
                    rospy.Subscriber(topic,Int32,lambda msg,id=id,regname=regname: self.set_register(msg,id,regname) )
                    self.subscribers.append(topic)
            
            
            
            
        
    def enable(self,msg):
        self.chain.enable(msg.data)
            
    def set_register(self,msg,id,regname):
        v=msg.data
        self.chain.set_reg(id,regname,v)

    def set_register_si(self,msg,id,regname):
        v=msg.data
        self.chain.set_reg(id,regname,self.chain.from_si(id,regname,v))
        
    def get_register_si(self,id,regname):
        v=self.chain.get_reg(id,regname)
        return self.chain.to_si(id,regname,v)
        
