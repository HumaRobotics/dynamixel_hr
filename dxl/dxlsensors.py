#!/usr/bin/env python


# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

from dxlcore import *
from dxlregisters import *



class DxlSensor(DxlElement):
    def __init__(self):
        DxlElement.__init__(self)

    def is_motor(self):
        return False
    

class DxlSensorAXS1(DxlSensor):
    __metaclass__=ModelRegisteringMetaclass
    model_name="AXS1"
    model_number=13
    documentation_url="http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm"
    
    def __init__(self):
        DxlSensor.__init__(self)


        self.registers["model_number"]=         DxlRegisterWord(0x00,'r',eeprom=True)
        self.registers["firmware"]=             DxlRegisterByte(0x02,'r',eeprom=True)
        self.registers["id"]=                   DxlRegisterByte(0x03,'rw',eeprom=True)
        self.registers["baud_rate"]=            DxlRegisterByte(0x04,'rw',eeprom=True)
        self.registers["return_delay"]=         DxlRegisterByte(0x05,'rw',eeprom=True)
        
        self.registers["ir_left"]=              DxlRegisterByte(0x1A,'r')
        self.registers["ir_center"]=            DxlRegisterByte(0x1B,'r')
        self.registers["ir_right"]=             DxlRegisterByte(0x1C,'r')
        
        self.registers["light_left"]=           DxlRegisterByte(0x1D,'r')
        self.registers["light_center"]=         DxlRegisterByte(0x1E,'r')
        self.registers["light_right"]=          DxlRegisterByte(0x1F,'r')

        self.registers["ir_obstacle_detected"]= DxlRegisterByte(0x20,'r')
        self.registers["light_detected"]=       DxlRegisterByte(0x21,'r')
        
        self.registers["sound_data"]=           DxlRegisterByte(0x23,'r')
        self.registers["sound_data_max_hold"]=  DxlRegisterByte(0x24,'rw')
        self.registers["sound_detected_count"]= DxlRegisterByte(0x25,'rw')
        self.registers["sound_detected_time"]=  DxlRegisterWord(0x26,'rw')
        
        self.registers["buzzer_note"]=          DxlRegisterByte(0x28,'rw')
        self.registers["buzzer_ringing_time"]=  DxlRegisterByte(0x29,'rw')
        
        self.registers["registered"]=           DxlRegisterByte(0x2C,'rw')
        
        self.registers["ir_remocon_arrived"]=   DxlRegisterByte(0x2E,'r')
        
        self.registers["lock"]=                 DxlRegisterByte(0x2F,'rw')
        
        self.registers["remocon_rx_data"]=      DxlRegisterWord(0x30,'r')
        self.registers["remocon_tx_data"]=      DxlRegisterWord(0x32,'rw')
        
        self.registers["ir_detect_reference"]=  DxlRegisterByte(0x34,'rw')
        self.registers["light_detect_reference"]=DxlRegisterByte(0x35,'rw')
        
        self.sort()
                
                