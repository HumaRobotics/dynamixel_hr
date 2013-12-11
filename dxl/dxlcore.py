#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)


from collections import OrderedDict
import logging



class DxlException(Exception):                    pass
class DxlConfigurationException(DxlException):    pass
class DxlCommunicationException(DxlException):    pass


class Dxl:
    BROADCAST      = 0xFE
    
    CMD_PING       = 0x01
    CMD_READ_DATA  = 0x02
    CMD_WRITE_DATA = 0x03
    CMD_REG_WRITE  = 0x04
    CMD_ACTION     = 0x05
    CMD_RESET      = 0x06
    CMD_SYNC_WRITE = 0x83


def get_model_name(model_number):
    try:
        return DxlElement.DxlModels[model_number].model_name
    except:
        return "??? %d"%model_number

class ModelRegisteringMetaclass(type):
    def __new__(cls, name, bases, attrs):
        inst=type.__new__(cls, name,bases,attrs)        
        DxlElement.registerModel(inst.model_number,inst)        
        return inst
        


class DxlElement(object):
    DxlModels={}
    
    def __init__(self):
        self.registers=OrderedDict()        
    
    
    @classmethod
    def registerModel(cls,model_number,model_cls):
        if model_number not in cls.DxlModels.keys():
            cls.DxlModels[model_number]=model_cls
            logging.info( "Registered Dynamixel Element model %s (%d): "%(model_cls.model_name,model_number)+str(model_cls) )

    @classmethod
    def instantiateMotor(cls,id,model_number):
        if not model_number in cls.DxlModels.keys():
            raise DxlConfigurationException,"Cannot instantiate non registered element model %d on ID %d"%(model_number,id)
        mcls=cls.DxlModels[model_number]
        return mcls()
        

        
    def getRegisterCmd(self,name):
        if not name in self.registers.keys():
            raise DxlConfigurationException,"Model %s has no register called %s"%(name,self.model_name)
        r=self.registers[name]
        if not 'r' in r.mode:
            raise DxlConfigurationException,"Register %s is not readable"%(name)
        return (r.size,[Dxl.CMD_READ_DATA,r.address,r.size])

    def setRegisterCmd(self,name,value):
        if not name in self.registers.keys():
            raise DxlConfigurationException,"Model %s has no register called %s"%(self.model_name,name)
        r=self.registers[name]
        if not 'w' in r.mode:
            raise DxlConfigurationException,"Register %s is not writable"%(name)
        if r.size!=len(value):
            raise DxlConfigurationException,"Model %s register %s has size %d: passed size %d"%(self.model_name,name,r.size,len(value))
            
        return (0,[Dxl.CMD_WRITE_DATA,r.address]+value )

    def sort(self):
        self.registers = OrderedDict( sorted(self.registers.iteritems(), key=lambda x: x[1].address) )
        
    def baud_to_si(self,val):
        return int(2000000/(val+1))

    def si_to_baud(self,val):        
        return int(2000000/(val)-1)



