#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

from dxlchain import *

import sys
import serial
import time
import logging
from threading import Lock
import json
import array
from collections import OrderedDict

logging.basicConfig(level=logging.DEBUG)

    
        

if __name__ == "__main__":    
    chain=DxlChain("COM21", rate=3000000)
    #~ chain.reopen()
    chain.dump()

    conf="""
{
    "1": {
        "model_number": 12, 
        "firmware": 24, 
        "id": 1, 
        "baud_rate": 1, 
        "return_delay": 250, 
        "cw_angle_limit": 0, 
        "ccw_angle_limit": 1023, 
        "high_temp_limit": 70, 
        "low_voltage_limit": 60, 
        "high_voltage_limit": 140, 
        "max_torque": 1023, 
        "status_return_level": 2, 
        "alarm_led": 36, 
        "alarm_shutdown": 36, 
        "torque_enable": 1, 
        "led": 0, 
        "cw_compliance_margin": 1, 
        "ccw_compliance_margin": 1, 
        "cw_compliance_slope": 32, 
        "ccw_compliance_slope": 32, 
        "goal_pos": 100, 
        "moving_speed": 50, 
        "torque_limit": 1023, 
        "present_position": 800, 
        "present_speed": 0, 
        "present_load": 0, 
        "present_voltage": 123, 
        "present_temp": 39, 
        "registered": 0, 
        "moving": 0, 
        "lock": 0, 
        "punch": 32
    }
}
"""
    #~ chain.set_configuration(json.loads(conf))
    time.sleep(1)
    chain.set_reg(1,"torque_enable",1)
    chain.set_reg(1,"moving_speed",100)
    chain.set_reg(1,"goal_pos",100)
    time.sleep(1)
    chain.set_reg(1,"goal_pos",2800)
    time.sleep(1)
    chain.set_reg(1,"torque_enable",0)
    
    chain.dump()
    chain.close()
