#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

from dxlcore import *
from dxlregisters import *
from dxlmotors import *
from dxlsensors import *

import sys
import serial
import time
import logging
from threading import Lock
import json
import array
from collections import OrderedDict
from post_threading import Post


    
        
        
class DxlChain:
    """
    Manages a list of Dynamixel motors on the same serial link.
    Provides thread-safe access to the chain.

    Instantiate by passing the serial device (COMXX or /dev/ttyUSBX) and the baudrate to use.
    If the chain is unknown you can directly call get_motor_list to obtain the list of available motors.
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
        self.post=Post(self)


    # Low-level communication (Thread unsafe functions with _)
    
    def open(self):
        """Opens the serial port"""
        with self.lock:
            self._open()

    def _open(self):
        logging.info("Opening serial port %s at rate %d bps, timeout %f s"%(self.portname,self.rate,self.timeout))    
        self.port=serial.Serial(self.portname,self.rate,timeout=self.timeout)    

    def close(self):
        """Closes the serial port"""
        with self.lock:
            self._close()

    def _close(self):
        try:
            logging.info("Closing serial port %s"%(self.portname))
            self.port.close()
        except:
            logging.warning("Could not close port %s"%self.portname)
    
    def reopen(self,portname=None,rate=None,timeout=None):
        """Reopens the serial port"""
        if portname:
            self.portname=portname
        if rate:
            self.rate=rate
        if timeout:
            self.timeout=timeout
        with self.lock:
            self._close()
            self._open()

    def send(self,id,packet):
        """Sends an instruction packet to a given ID. Header, length and checksum will be automatically added"""
        with self.lock:
            self._send(id,packet)
            
    def _send(self, id, packet):
        """ Takes a payload, packages it as [header,id,length,payload,checksum], sends it on serial and flush"""
        checksumed_data = [id, len(packet)+1] + packet
        
        data="".join(map(chr, [0xFF, 0xFF] + checksumed_data + [self.checksum(checksumed_data)]))
        self.port.write(data)
        self.port.flushOutput()

    def recv(self):
        """Wait for a response on the serial, validate it, raise errors if any, return id and data if any"""
        with self.lock:
            return self._recv()

    def _recv(self):
        """Wait for a response on the serial, validate it, raise errors if any, return id and data if any """
        # Read the first 4 bytes 0xFF,0xFF,id,length
        header = array.array('B',self.port.read(4))
        if(len(header)!=4):
            raise DxlCommunicationException('Could not read first 4 bytes of expected response, got %d bytes'%len(header))
        else :
            id,expectedsize=header[2:4]
            # Read number of expected bytes
            data=array.array('B',self.port.read(expectedsize))
            if len(data)!=expectedsize:
                raise DxlCommunicationException('Could not read %d data bytes of expected response, got %d bytes'%(expectedsize,len(data)))
                
            if len(data)>0:
                error=data[0]

                if error!=0 and error!=2: # skip angle errors
                    # TODO Distinguish communication/Hardware errors
                    raise DxlCommunicationException('Received error code from motor %d: %d'%(id,error))
                
                checksum=self.checksum(header[2:]+data[:-1])
                if checksum!=data[-1]:
                    raise DxlCommunicationException('Invalid checksum')
                data=data[1:-1]
                #~ print data
            return (id,data)

    def comm(self,id,packet):
        """Communicate with the Dynamixel by sending a packet and retrieving the response"""
        with self.lock:
            return self._comm(id,packet)

    def _comm(self,id,packet):
        self._send(id,packet)
        return self._recv()

    def checksum(self,values):
        """Compute packet checksum."""
        return (~sum(values))&0xFF



    # Basic commands
    
    def ping(self,id):
        """Sends a ping packet to a motor, raises an exception if no answer is received"""
        with self.lock:
            self._ping(id)
            
    def _ping(self,id):
        """Sends a ping packet to a motor, raises an exception if no answer is received"""
        self._send(id,[Dxl.CMD_PING])
        self._recv()

    def factory_reset(self,id):
        """Performs a factory reset on a motor"""
        with self.lock:
            self._factory_reset(id)
            
    def _factory_reset(self,id):
        """Performs a factory reset on a motor"""
        self._send(id,[Dxl.CMD_RESET])
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
        """Read data from the registers of a motor"""
        data=self._comm(id,[Dxl.CMD_READ_DATA,address,size])[1]
        if len(data)!=size:
            raise DxlCommunicationException('Read command did not obtain the %d bytes expected: got %d bytes'%(size,len(data)))
        return data

    def _write(self,id,address,values):        
        """Write data to a motor registers"""
        self._comm(id,[Dxl.CMD_WRITE_DATA,register,values])


    def get_model_number(self,id):
        """Obtain the model number of a motor"""
        with self.lock:
            return self._get_model(id)
            
    def _get_model(self,id):
        """Obtain the model number of a motor"""
        data=self._read(id,0,2)
        return (data[1]<<8)+data[0]

        
    # Register Access

    def get_reg(self,id,name):
        """Read a named register from a motor"""
        if id not in self.motors.keys():
            raise DxlConfigurationException,'Motor ID %d does not exist on the chain'%(id) 
        m=self.motors[id]
        reg=m.registers[name]
        (esize,cmd)=m.getRegisterCmd(name)
        (nid,data)=self.comm(id,cmd)
        if len(data)!=esize:
            raise DxlCommunicationException,'Motor ID %d did not retrieve expected register %s size %d: got %d bytes'%(id,name,esize,len(data)) 
        v=reg.fromdxl(data)
        logging.info('Motor ID %d get register %s: %d'%(id,name,v) )
        return v

    def get_reg_si(self,id,name):
        """Read a named register from a motor and returns value converted to SI units"""
        if id not in self.motors.keys():
            raise DxlConfigurationException,'Motor ID %d does not exist on the chain'%(id) 
        m=self.motors[id]
        reg=m.registers[name]
        (esize,cmd)=m.getRegisterCmd(name)
        (nid,data)=self.comm(id,cmd)
        if len(data)!=esize:
            raise DxlCommunicationException,'Motor ID %d did not retrieve expected register %s size %d: got %d bytes'%(id,name,esize,len(data)) 
        v=reg.fromdxl(data)
        logging.info('Motor ID %d get register %s: %d'%(id,name,v) )
        return reg.tosi(v)

    def set_reg(self,id,name,v):
        """Sets a named register on a motor"""
        if id not in self.motors.keys():
            raise DxlConfigurationException,'Motor ID %d does not exist on the chain'%(id) 
        m=self.motors[id]
        reg=m.registers[name]
        (esize,cmd)=m.setRegisterCmd(name,reg.todxl(v))
        (nid,data)=self.comm(id,cmd)
        logging.info('Motor ID %d set register %s to %d'%(id,name,v) )
        if len(data)!=esize:        
            raise DxlCommunicationException,'Motor ID %d did not retrieve expected register %s size %d: got %d bytes'%(id,name,esize,len(data)) 

    def set_reg_si(self,id,name,v):
        """Sets a named register on a motor using SI units"""
        if id not in self.motors.keys():
            raise DxlConfigurationException,'Motor ID %d does not exist on the chain'%(id) 
        m=self.motors[id]
        reg=m.registers[name]
        (esize,cmd)=m.setRegisterCmd(name,reg.todxl(reg.fromsi(v)))
        (nid,data)=self.comm(id,cmd)
        logging.info('Motor ID %d set register %s to %d'%(id,name,v) )
        if len(data)!=esize:        
            raise DxlCommunicationException,'Motor ID %d did not retrieve expected register %s size %d: got %d bytes'%(id,name,esize,len(data)) 

    
    def sync_write_pos_speed(self,ids,positions,speeds): 
        """Performs a synchronized write of 'goal_pos' and 'moving_speed' registers for a set of motors (if possible)"""
        regpos=None
        regspeed=None
        # Check motor IDs, goal_pos and moving_speed register address and sizes
        for id in ids:
            if id not in self.motors.keys():
                raise DxlConfigurationException,"Motor ID %d cannot be found in chain"%id
            m=self.motors[id]
            reg_name="goal_pos"
            if reg_name not in m.registers.keys():
                raise DxlConfigurationException,"Synchronized write %s impossible on chain, register absent from motor ID %d"%(reg_name,id)
            r=m.registers[reg_name]
            if regpos==None:
                regpos=r
            else:
                if regpos.address!=r.address:
                    raise DxlConfigurationException,"Synchronized write %s impossible on chain, mismatch in register address for motor ID %d"%(reg_name,id)
                if regpos.size!=r.size:
                    raise DxlConfigurationException,"Synchronized write %s impossible on chain, mismatch in register size for motor ID %d"(reg_name,id)

            reg_name="moving_speed"
            if reg_name not in m.registers.keys():
                raise DxlConfigurationException,"Synchronized write %s impossible on chain, register absent from motor ID %d"%(reg_name,id)
            r=m.registers[reg_name]
            if regspeed==None:
                regspeed=r
            else:
                if regspeed.address!=r.address:
                    raise DxlConfigurationException,"Synchronized write %s impossible on chain, mismatch in register address for motor ID %d"%(reg_name,id)
                if regspeed.size!=r.size:
                    raise DxlConfigurationException,"Synchronized write %s impossible on chain, mismatch in register size for motor ID %d"(reg_name,id)
                
        if (regpos.address+regpos.size)!=regspeed.address:
            raise DxlConfigurationException,"Synchronized write goal_pos/moving_speed impossible on chain, registers are not consecutive"
            
        # Everything is ok, build command and send
        payload= [Dxl.CMD_SYNC_WRITE,regpos.address,regpos.size+regspeed.size]
        for i in range(0,len(ids)):
            id=ids[i]
            pos=positions[i]
            speed=speeds[i]
            payload.append(id)
            payload.extend(regpos.todxl(pos))
            payload.extend(regspeed.todxl(speed))
            
        self.send(Dxl.BROADCAST,payload) 
            


    
    
    def sync_write_pos(self,ids,positions):
        """Performs a synchronized write of 'goal_pos' register for a set of motors (if possible)"""
        reg=None
        # Check motor IDs, goal_pos and moving_speed register address and sizes
        for id in ids:
            if id not in self.motors.keys():
                raise DxlConfigurationException,"Motor ID %d cannot be found in chain"%id
            m=self.motors[id]
            reg_name="goal_pos"
            if reg_name not in m.registers.keys():
                raise DxlConfigurationException,"Synchronized write %s impossible on chain, register absent from motor ID %d"%(reg_name,id)
            r=m.registers[reg_name]
            if reg==None:
                reg=r
            else:
                if reg.address!=r.address:
                    raise DxlConfigurationException,"Synchronized write %s impossible on chain, mismatch in register address for motor ID %d"%(reg_name,id)
                if reg.size!=r.size:
                    raise DxlConfigurationException,"Synchronized write %s impossible on chain, mismatch in register size for motor ID %d"(reg_name,id)
                
        # Everything is ok, build command and send
        payload= [Dxl.CMD_SYNC_WRITE,reg.address,reg.size]
        for i in range(0,len(ids)):
            id=ids[i]
            pos=positions[i]
            payload.append(id)
            payload.extend(reg.todxl(pos))
            
        self.send(Dxl.BROADCAST,payload) 
    
    def to_si(self,id,name,v):        
        """Converts a motor register value from dynamixel format to SI units"""
        reg=self.motors[id].registers[name]
        return reg.tosi(v)

    def from_si(self,id,name,v):
        """Converts a motor register value from SI units to dynamixel format"""
        reg=self.motors[id].registers[name]
        return reg.fromsi(v)

    # Configuration get/set functionalities
    
    def get_motor_list(self,broadcast=True,instantiate=True):
        with self.lock:
            """Obtains the list of motors available on a chain, and optionally tries to instantiante them."""
            start=time.time()
            self.motors={}
            if broadcast:
                ids=self._ping_broadcast()
            else:
                ids=range(0,Dxl.BROADCAST)
            logging.info("Scanning IDs :"+str(ids))
            l=[]            
            for id in ids:
                if not broadcast:
                    try:
                        self._ping(id)
                    except:
                        continue
                model=self._get_model(id)
                logging.info("Found motor ID %d model %d"%(id,model))
                l.append(id)
                if instantiate:
                    m=DxlMotor.instantiateMotor(id,model)
                    self.motors[id]=m
                    logging.info("Instantiated motor ID %d model %s (%d)"%(id,m.model_name,model))
            delay=time.time()-start
            logging.debug("get_motor_list delay: %f"%delay)
            return l

    def get_configuration(self,broadcast=True):
        """Obtain the list of motors on a chain, read and return their full configuration"""
        self.get_motor_list(broadcast=broadcast)
        start=time.time()
        d=OrderedDict()
        for (id,m) in self.motors.items():
            dd=OrderedDict()
            d[id]=dd
            for (name,r) in m.registers.items():
                dd[name]=self.get_reg(id,name)
        delay=time.time()-start
        logging.debug("get_configuration reg delay: %f"%delay)
        return d
        
    def set_configuration(self,conf):
        """Compare the motors available on a chain to those specified in the provided configuration, silently try to set all registers, generates an exception if there is a mismatch"""
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

            # Check/Set all registers
            for (name,val) in conf[sid].items():
                if name not in motor.registers.keys(): raise DxlConfigurationException,"Cannot configure missing register %s on motor ID %d"%(name,iid)                    
                reg=motor.registers[name]
                current=self.get_reg(iid,name)
                if current==val: continue
                # Value has to be changed
                if not 'w' in reg.mode: # read only: generate error if setting is EEPROM
                    if reg.eeprom: raise DxlConfigurationException,"Invalid EEPROM value in motor ID %d register %s: current=%d expected=%d"%(iid,name,current,val)
                    else: pass
                else: # Set value
                    if reg.eeprom:
                        logging.info( "Changed (EEPROM register %s from %d to %d on motor ID %d"%(name,current,val,iid) )
                    self.set_reg(iid,name,val) # if writable set it

    def dump(self):
        """Obtain the motors chain configuration and dumps it on stdout"""
        conf=self.get_configuration()
        print json.dumps(conf,indent=4,sort_keys=False)

    def get_motors(self,ids=None):
        """Return the list of all motors ids, or a specific set, or a single id"""        
        if ids==None:
            return self.motors.keys()
            
        elif type(ids)==type(list()):
            return ids
        elif type(ids)==type(int()):
            return [ids]
        raise Exception,"Invalid type for motor id: %s"%str(ids)
        
            
            
    def enable(self,ids=None):
        """Enable or disable all the motors on the chain"""
        ids=self.get_motors(ids)
        for id in ids:
            self.set_reg(id,"torque_enable",1)
    
    def disable(self,ids=None):
        """Disable all the motors on the chain"""
        ids=self.get_motors(ids)
        for id in ids:
            self.set_reg(id,"torque_enable",0)
        
    def wait_stopped(self,ids=None):
        """
        Wait for a specific motor or all motors on the chain to have stopped moving.
        This is a blocking function.
        """
        ids=self.get_motors(ids)
        while True:
            moving=False
            for id in ids:
                if self.get_reg(id,"moving")!=0:
                    moving=True
                    break
            if not moving:
                break
            time.sleep(0.1)

    def is_moving(self,ids=None):
        """Returns True is a specific motor (or any motor on the chain) is still moving."""
        ids=self.get_motors(ids)
        
        for id in ids:
            if self.get_reg(id,"moving")!=0:
                return True
        return False
                
    def goto(self,id,pos,speed=None,blocking=True):
        """
        Moves a motor to a position at a specified speed (or current speed if none provided) and waits for motion to be completed (unless blocking=False is passed).
        This is a blocking function by default.
        """
        if speed!=None:
            self.set_reg(id,"moving_speed",speed)
        self.set_reg(id,"goal_pos",pos)
        if blocking:
            self.wait_stopped(ids=id)        
            
    def get_position(self,ids=None):
        """Obtain the current position from the chain or a set of motors"""
        ids=self.get_motors(ids)
        pos=dict()
        for id in ids:
            pos[id]=self.get_reg(id,"present_position")
        return pos
    
    def set_position(self,pos,blocking=True):
        """
        Set the position of a set of motors by providing an ID indexed dictionary
        This is a blocking function by default
        """
        self.sync_write_pos(pos.keys(),pos.values())
        if blocking:
            self.wait_stopped()
        

    def save_position(self,filename,ids=None):
        """Obtain the current pose from the chain or a set of motors and saves it in a JSON formatted file"""
        ids=self.get_motors(ids)
        pos=self.get_position(ids)
        txt=json.dumps(pos,indent=4,sort_keys=False)
        f=open(filename,'w')
        f.write(txt)
        f.close()

    def load_position(self,filename,blocking=True):
        """
        Set the position of a set of motors by providing a JSON formatted file.
        This is a blocking function by default.
        """
        f=open(filename,'r')
        txt=f.read()
        f.close()
        d=json.loads(txt)
        pos=dict()
        for k,v in d.items():
            pos[int(k)]=v            
        self.set_position(pos,blocking)
        
