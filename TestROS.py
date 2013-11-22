import logging
logging.basicConfig(level=logging.DEBUG)


import roslib
import rospy
from dxl import *
from dxl import dxlros


if __name__=="__main__":
        rospy.init_node("dxl")
        chain=dxlchain.DxlChain("/dev/ttyUSB0",rate=1000000)
        chain.get_motor_list()
        bindings={1:"pan",2:"tilt1",3:"tilt2",4:"tilt3",5:"head"}
        dxlros=dxlros.DxlROS(chain,rate=10,bindings=bindings)        
        rospy.spin()
