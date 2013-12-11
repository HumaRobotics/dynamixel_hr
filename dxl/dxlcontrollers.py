#!/usr/bin/env python


# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

from dxlcore import *
from dxlregisters import *



class DxlController(DxlElement):
    def __init__(self):
        DxlElement.__init__(self)

    def is_motor(self):
        return False
    

class DxlControllerCM730(DxlController):
    __metaclass__=ModelRegisteringMetaclass
    model_name="CM730"
    model_number=29440
    documentation_url="http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/sub_controller_%28cm-730%29.htm"
    
    def __init__(self):
        DxlController.__init__(self)


        self.registers["model_number"]=         DxlRegisterWord(0x00,'r',eeprom=True)
        self.registers["firmware"]=             DxlRegisterByte(0x02,'r',eeprom=True)
        self.registers["id"]=                   DxlRegisterByte(0x03,'rw',eeprom=True)
        self.registers["baud_rate"]=            DxlRegisterByte(0x04,'rw',eeprom=True)
        self.registers["return_delay"]=         DxlRegisterByte(0x05,'rw',eeprom=True)
        
        self.registers["dynamixel_power"]=         DxlRegisterByte(0x18,'rw')
        self.registers["gyro_z"]=         DxlRegisterWord(0x26,'r')
        self.registers["gyro_y"]=         DxlRegisterWord(0x28,'r')
        self.registers["gyro_x"]=         DxlRegisterWord(0x2A,'r')        
        
        self.registers["acc_x"]=         DxlRegisterWord(0x2C,'r')
        self.registers["acc_y"]=         DxlRegisterWord(0x2E,'r')
        self.registers["acc_z"]=         DxlRegisterWord(0x30,'r')        
        
        self.sort()
                
                