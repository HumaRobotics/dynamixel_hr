#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

        

class DxlRegister():
    
    def __init__(self,address,size,mode='r',eeprom=False,fromdxl= lambda x: x,todxl= lambda x: x,fromsi=lambda x:x,tosi=lambda x:x,range=None):
        self.address=address
        self.size=size
        self.mode=mode
        self.eeprom=eeprom
        self.fromdxl=fromdxl
        self.todxl=todxl
        self.fromsi=fromsi
        self.tosi=tosi
        self.range=range

class DxlRegisterByte(DxlRegister):
    def __init__(self,address,mode='r',eeprom=False,fromsi=lambda x:x,tosi=lambda x:x,range=None):
        DxlRegister.__init__(self,address,1,mode,eeprom,fromdxl=lambda x:x[0],todxl=lambda x:[x],range=range,fromsi=fromsi,tosi=tosi)

class DxlRegisterWord(DxlRegister):
    def __init__(self,address,mode='r',eeprom=False,fromsi=lambda x:x,tosi=lambda x:x,range=None):
        DxlRegister.__init__(self,address,2,mode,eeprom,fromdxl=lambda x:x[0]+(x[1]<<8),todxl=lambda x:[int(x)&0xFF,(int(x)>>8)&0xFF] ,range=range,fromsi=fromsi,tosi=tosi)

