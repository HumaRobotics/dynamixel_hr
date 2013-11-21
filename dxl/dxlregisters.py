#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

        

class DxlRegister():
    
    def __init__(self,address,size,mode='r',eeprom=False,fromdxl= lambda x: x,todxl= lambda x: x):
        self.address=address
        self.size=size
        self.mode=mode
        self.eeprom=eeprom
        self.fromdxl=fromdxl
        self.todxl=todxl

class DxlRegisterByte(DxlRegister):
    def __init__(self,address,mode='r',eeprom=False):
        DxlRegister.__init__(self,address,1,mode,eeprom,fromdxl=lambda x:x[0],todxl=lambda x:[x])

class DxlRegisterWord(DxlRegister):
    def __init__(self,address,mode='r',eeprom=False):
        DxlRegister.__init__(self,address,2,mode,eeprom,fromdxl=lambda x:x[0]+(x[1]<<8),todxl=lambda x:[x&0xFF,(x>>8)&0xFF] )

