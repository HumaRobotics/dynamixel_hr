#!/usr/bin/env python

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
    
    def __init__(self,chain,rate=10,motors=None):
        Thread.__init__(self)
        self.chain=chain
        self.rate=rate
        self.motors=motors

        self.publishers=[]
        self.subscribers=[]        
        self.pub={}
        
        self.do_stop=False
        
        if self.motors==None: # Use IDs as names if no binding provided
            self.motors=self.chain.motors.keys()
        else:
            for id  in self.motors:
                if id not in self.chain.motors.keys():
                    raise Exception,"Cannot bind ROS name %s to non-existing motor ID %d"%(name,id)

        logging.info("Creating ROS elements")
        
        
        self.buildPublishers()
        self.buildSubscribers()
        
        self.start()

    def create_subscriber(self,topic,type,callback):
        logging.info("Creating subscriber on ROS topic %s"%topic)
        rospy.Subscriber(topic,type,callback)
        self.subscribers.append(topic)

    def create_publisher(self,topic,type):
        logging.info("Creating publisher on ROS topic %s"%topic)
        self.pub[topic]=rospy.Publisher(topic,type)
        self.publishers.append(topic)
        
        
    def run(self):
        r=rospy.Rate(float(self.rate))
        while not rospy.is_shutdown() and not self.do_stop:
            self.publish()
            r.sleep()
        
    def publish(self):
        data=[]
        for id in self.motors:
            v=self.chain.get_reg_si(id,"present_position")
            data.append(v)
        
        msg=Float64MultiArray()
        msg.data=data
        self.pub["/dxl/present_position"].publish(msg) 
    
        
        
    def stop(self):
        self.do_stop=True
    
    def buildPublishers(self):
        self.create_publisher("/dxl/present_position",Float64MultiArray)

                
    
    def buildSubscribers(self):
        self.create_subscriber("/dxl/enable",Bool,self.enable)
            
                        
        
    def enable(self,msg):
        if msg.data==True:
            self.chain.enable(self.bindings.keys())
        else:
            self.chain.disable(self.bindings.keys())