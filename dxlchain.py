#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

from dxlcore import *
from dxlregisters import *
from dxlmotors import *

import sys
import serial
import time
import logging
from threading import Lock
import json
import array
from collections import OrderedDict

    
        
        
class DxlChain:
    """
    Manages a list of Dynamixel motors on the same serial link
    Provides thread-safe access to the chain
    """

    def __init__(self, portname,rate=57142,timeout=0.04):
        """
        DO NOT CHANGE THE DEFAULT BAUDRATE HERE: 57142 is the factory setting of Dynamixel motors
        """
        self.portname=portname
        self.rate=rate
        self.timeout=timeout
        self.lock=Lock()
        self.open()
        self.configuration=None
        self.motors={}
    


    # Low-level communication (Thread unsafe functions with _)
    
    def open(self):
        with self.lock:
            self._open()

    def _open(self):
        logging.info("Opening serial port %s at rate %d bps, timeout %f s"%(self.portname,self.rate,self.timeout))    
        self.port=serial.Serial(self.portname,self.rate,timeout=self.timeout)    

    def close(self):
        with self.lock:
            self._close()

    def _close(self):
        try:
            logging.info("Closing serial port %s"%(self.portname))
            self.port.close()
        except:
            logging.warning("Could not close port %s"%self.portname)
    
    def reopen(self):
        with self.lock:
            self._close()
            self._open()

    def send(self,id,packet):
        with self.lock:
            self._send(id,packet)
            
    def _send(self, id, packet):
        """ Takes a payload, packages it as [header,id,length,payload,checksum], sends it on serial and flush"""
        checksumed_data = [id, len(packet)+1] + packet
        
        data="".join(map(chr, [0xFF, 0xFF] + checksumed_data + [self.checksum(checksumed_data)]))
        self.port.write(data)
        self.port.flushOutput()

    def recv(self):
        with self.lock:
            return self._recv()

    def _recv(self):
        """ Wait for a response on the serial, validate it, raise errors if any, return id and data if any """
        # Read the first 4 bytes 0xFF,0xFF,id,length
        header = array.array('B',self.port.read(4))
        if(len(header)!=4):
            raise DxlCommunicationException('Could not read first 4 bytes of expected response, got %d bytes'%len(header))
        else :
            id,expectedsize=header[2:4]
            # Read number of expected bytes
            data=array.array('B',self.port.read(expectedsize))
            if len(data)!=expectedsize:
                raise DxlCommunicationException('Could not read %d data bytes of expected response, got %d bytes'%(len(expectedsize),len(data)))
                
            error=data[0]
            if error!=0:
                # TODO Distinguish communication/Hardware errors
                raise DxlCommunicationException('Received error code from motor %d: %d'%(id,error))
            
            checksum=self.checksum(header[2:]+data[:-1])
            if checksum!=data[-1]:
                raise DxlCommunicationException('Invalid checksum')
            data=data[1:-1]
            #~ print data
            return (id,data)

    def comm(self,id,packet):
        with self.lock:
            return self._comm(id,packet)

    def _comm(self,id,packet):
        self._send(id,packet)
        return self._recv()

    def checksum(self,values):
        """ Compute packet checksum """
        return (~sum(values))&0xFF



    # Basic commands
    
    def _ping(self,id):
        self._send(id,[Dxl.CMD_PING])
        self._recv()
        
    def _ping_broadcast(self):
        """ Perform a ping on all servos. Returns list of responding IDs """
        self._send(Dxl.BROADCAST,[Dxl.CMD_PING])
        l=[]
        while True:
            try:
                (id,data)=self._recv()
                l.append(id)
            except DxlCommunicationException:
                break            
        return l


    def _read(self,id,address,size):
        data=self._comm(id,[Dxl.CMD_READ_DATA,address,size])[1]
        if len(data)!=size:
            raise DxlCommunicationException('Read command did not obtain the %d bytes expected: got %d bytes'%(size,len(data)))
        return data

    def _write(self,id,address,values):        
        self._comm(id,[Dxl.CMD_WRITE_DATA,register,values])


    def _get_model(self,id):
        data=self._read(id,0,2)
        return (data[1]<<8)+data[0]

        
    # Register Access

    def get_reg(self,id,name):
        m=self.motors[id]
        reg=m.registers[name]
        (esize,cmd)=m.getRegisterCmd(name)
        (nid,data)=self.comm(id,cmd)
        if len(data)!=esize:
            raise DxlCommunicationException,'Motor ID %d did not retrieve expected register %s size %d: got %d bytes'%(id,name,esize,len(data)) 
        return reg.fromdxl(data)

    def set_reg(self,id,name,v):
        m=self.motors[id]
        reg=m.registers[name]
        (esize,cmd)=m.setRegisterCmd(name,reg.todxl(v))
        (nid,data)=self.comm(id,cmd)
        logging.info('Motor ID %d set register %s to %d'%(id,name,v) )
        if len(data)!=esize:        
            raise DxlCommunicationException,'Motor ID %d did not retrieve expected register %s size %d: got %d bytes'%(id,name,esize,len(data)) 

    
        
    # Configuration get/set functionalities
    
    def get_motor_list(self):
        self.motors={}
        ids=self._ping_broadcast()
        for id in ids:
            model=self._get_model(id)
            logging.info("Found motor ID %d model %d"%(id,model))
            m=DxlMotor.instantiateMotor(model)
            self.motors[id]=m
            logging.info("Instantiated motor ID %d model %s (%d)"%(id,m.model_name,model))


    def get_configuration(self):
        self.get_motor_list()
        d=OrderedDict()
        for (id,m) in self.motors.items():
            dd=OrderedDict()
            d[id]=dd
            for (name,r) in m.registers.items():
                dd[name]=self.get_reg(id,name)
        return d
        
    def set_configuration(self,conf):
        d={}
        self.get_motor_list()
        for id in conf.keys():
            sid=id
            iid=int(sid)
            if iid not in self.motors.keys(): raise DxlConfigurationException,"Cannot find motor ID %d to be configured"%iid
            motor=self.motors[iid]

            # Validate EEPROM read-only settings
            for (name,val) in conf[sid].items():                                
                if name not in motor.registers.keys(): continue
                reg=motor.registers[name]
                current=self.get_reg(iid,name)
                if current==val: continue
                # Value has to be changed
                if not 'w' in reg.mode: # read only: generate error if setting is EEPROM
                    if reg.eeprom: raise DxlConfigurationException,"Invalid EEPROM value in motor ID %d register %s: current=%d expected=%d"%(iid,name,current,val)
                    else: pass

            for (name,val) in conf[sid].items():
                if name not in motor.registers.keys(): raise DxlConfigurationException,"Cannot configure missing register %s on motor ID %d"%(name,iid)                    
                reg=motor.registers[name]
                current=self.get_reg(iid,name)
                if current==val: continue
                # Value has to be changed
                if not 'w' in reg.mode: # read only: generate error if setting is EEPROM
                    if reg.eeprom: raise DxlConfigurationException,"Invalid EEPROM value in motor ID %d register %s: current=%d expected=%d"%(iid,name,current,val)
                    else: pass
                else:
                    if reg.eeprom:
                        logging.info( "Changed (EEPROM register %s from %d to %d on motor ID %d"%(name,current,val,iid) )
                    self.set_reg(iid,name,val) # if writable set it

    def dump(self):
        conf=self.get_configuration()
        print json.dumps(conf,indent=4,sort_keys=False)
