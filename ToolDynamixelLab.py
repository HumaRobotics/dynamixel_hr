#!/usr/bin/python
import logging
logging.basicConfig(level=logging.DEBUG)

import os
import time
import json
from string import *

from Tkinter import *
import tkMessageBox
from serial import SerialException


from dxl import *

searchRates=[57142,3000000,1000000,9600]



#~ class MotorWindow:

    #~ def __init__(self, master,parent,motor,id):
        #~ self.master=master
        #~ self.parent=parent
        #~ self.motor=motor
        #~ self.id=id
        
        #~ self.row=1
        
        #~ self.window=Toplevel(self.master)
        #~ self.window.title("Motor %d"%id)
        #~ self.window.bind('<Key-Escape>', self.destroy )
        #~ self.frame=Frame(self.window, width= 300, height= 200)
        
        #~ self.addRegister("goal_pos")
        #~ self.addRegister("moving_speed")
        
        #~ self.frame.pack()
        
    #~ def addRegister(self,register):
        #~ Label(self.frame,text=register).grid(column=0,row=self.row)
        #~ val=self.parent.chain.get_reg(self.id,register)
        #~ scale=Scale(self.frame, from_=0, to=4095, orient=HORIZONTAL,command=lambda val,register=register: self.set(register,val))
        #~ scale.set(val)
        #~ scale.grid(column=0,row=self.row+1)
        #~ self.row+=2
        
    #~ def destroy(self):
        #~ del self.parent.motorWindows[self.id]
        #~ self.window.destroy()
    
    #~ def set(self,register,value):
        #~ print str(register)+" "+str(value)
        #~ self.parent.chain.set_reg(self.id,register,int(value))




class MotorsWindow:

    def __init__(self, master,parent):
        self.master=master
        self.parent=parent
        self.chain=parent.chain
        
        self.window=Toplevel(self.master)
        self.window.title("Motors")        

        self.window.protocol("WM_DELETE_WINDOW", self.destroy)
        self.window.bind('<Key-Escape>', self.destroy )
        
        self.frame=Frame(self.window, width= 300, height= 200)
        
        self.row=0
        self.column=0
        self.update=False
        for id in self.chain.motors.keys():
            self.generate(id)            
        
        self.frame.pack()
        self.master.after(500,self.startUpdating)

    def startUpdating(self):
        self.update=True
        
    def generate(self,id): 
        model_name=self.chain.motors[id].model_name
        Label(self.frame,text="MOTOR %d %s"%(id,model_name)).grid(column=self.column,row=self.row,columnspan=2)
        self.row+=1
        motor=self.chain.motors[id]
        #~ for rname,reg in motor.registers.items():
            #~ if 'w' in reg.mode and not reg.eeprom:
                #~ self.addRegister(id,rname)
        
        self.addRegister(id,"goal_pos")        
        self.addRegister(id,"moving_speed")
        #~ self.addRegister(id,"p_gain")
        if self.row>10:
            self.row=0
            self.column+=2

    def addRegister(self,id,register):
        Label(self.frame,text=register).grid(column=self.column,row=self.row)
        val=self.chain.get_reg(id,register)
        reg=self.chain.motors[id].registers[register]
        
        if reg.range:
            range=reg.range
        elif reg.size==1:
            range=[0,255]
        else:
            range=[0,65535]
        
        scale=Scale(self.frame, from_=range[0], to=range[1], length=250,orient=HORIZONTAL)        
        scale.set(val)
        scale.configure( command=lambda val,id=id,register=register: self.set(id,register,val) )
        scale.grid(column=self.column+1,row=self.row)
        self.row+=1
        
    def destroy(self):
        self.parent.motorsWindow=None
        self.window.destroy()
    
    def set(self,id,register,value):
        if self.update:
            self.parent.chain.set_reg(id,register,int(value))
        
        
class MainWindow:

    def __init__(self, master):
        self.master=master

        self.motorsWindow=None
        self.chain=None

        self.width=800
        self.height=600
        self.frame = Frame(master, width= self.width, height= self.height)
        self.master.bind('<Key-Escape>', self.exit )

        #~ Label(self.frame, text="WARNING: use with 1 servo connected at a time !!").pack()

        Label(self.frame,text="Port:").grid(column=0,row=0)
        
        self.comPort= StringVar()
        if os.name=="nt":
            port="COM21"
        else:
            port="/dev/ttyUSB0"
        self.comPort.set(port)
        entryComPort = Entry(self.frame, textvariable=self.comPort)
        entryComPort.grid(column=1,row=0)
        
        Button(self.frame,text="Scan",command=self.scan).grid(column=2,row=0)

        Label(self.frame,text="Baudrate:").grid(column=3,row=0)
        self.baudRate= IntVar()
        self.baudRate.set("1000000")
        entryBaudRate= Entry(self.frame, textvariable=self.baudRate)
        entryBaudRate.grid(column=4,row=0)
        
        Button(self.frame,text="Connect",command=self.connect).grid(column=5,row=0)



        # Text field for configuration with scrollbars
        self.textConfig=Text(self.frame,width=50,height=30)
        self.textConfig.grid(column=0,row=2,columnspan=10,rowspan=10)
        self.textConfig.insert(END,"{}")
        
        scrly = Scrollbar(self.frame, command=self.textConfig.yview)
        self.textConfig.config(yscrollcommand=scrly.set)
        scrly.grid(column=10,row=2,rowspan=10,sticky="ns")
        
        scrlx = Scrollbar(self.frame, command=self.textConfig.xview,orient=HORIZONTAL)
        self.textConfig.config(xscrollcommand=scrlx.set)
        scrlx.grid(column=0,row=12,columnspan=10,sticky="ew")
        


        Button(self.frame,text="Refresh",command=self.refresh).grid(column=11,row=5)
        Button(self.frame,text="Set",command=self.set).grid(column=11,row=6)
        Button(self.frame,text="Activate",command=self.activate).grid(column=11,row=7)
        Button(self.frame,text="Deactivate",command=self.deactivate).grid(column=11,row=8)

        Button(self.frame,text="Show Motors",command=lambda: self.createMotorsWindow()).grid(column=11,row=9)
        
        
        self.frame.pack()
        
        
    def createMotorsWindow(self):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        if self.motorsWindow==None:
            self.motorsWindow=MotorsWindow(self.master,self)
        
        
    def exit(self,event=None):
        self.close()
        self.master.destroy()

    def open(self,rate):
        self.close()
        comPort=self.comPort.get()
        self.chain=dxlchain.DxlChain(comPort,rate=rate)

    def close(self):
        if self.motorsWindow:
            self.motorsWindow.destroy()
            self.motorsWindow=None
        if self.chain:
            try:
                self.chain.close()
                self.chain=None
            except:
                print "WARNING: could not close chain"
        
    def scan(self):
        selected_rate=None
        for rate in searchRates:            
            try:
                self.open(rate)
            except SerialException,e:
                tkMessageBox.showerror("Serial Error","Could not open serial port: \n"+str(e))
                return
                
            try:
                motors=self.chain.get_motor_list(instantiate=False)                
                print "rate %d: %s"%(rate,str(motors))
                if len(motors)>0:
                    selected_rate=rate
                #~ chain.dump()
            finally:
                self.close()
        
        if selected_rate:
            self.selectRate(selected_rate)

    def connect(self):
        self.selectRate(self.baudRate.get())


    def set(self):
        if self.chain:
            txt=self.textConfig.get(1.0,END)
            try:
                self.conf=json.loads(txt)
            except Exception,e:
                tkMessageBox.showerror("JSON Error","Could not parse JSON formatted configuration: \n"+str(e))
                return
                
            try:
                self.chain.set_configuration(self.conf)
            except Exception,e:
                tkMessageBox.showerror("Configuration Error","Could not set Dynamixel configuration: \n"+str(e))
                return
        else:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")

    def refresh(self):
        if self.chain:
            self.conf=self.chain.get_configuration()
            self.showConfig(self.conf)
        else:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")


    def selectRate(self,rate):
        print "Selected rate %d"%rate
        self.baudRate.set(rate)
        try:
            self.open(rate)
        except SerialException,e:
            tkMessageBox.showerror("Serial Error","Could not open serial port: \n"+str(e))
            return
        self.conf=self.chain.get_configuration()
        #~ self.chain.dump()
        self.showConfig(self.conf)

    def showConfig(self,config):
        txt=json.dumps(config,indent=4,sort_keys=False)
        self.textConfig.delete(1.0,END)            
        self.textConfig.insert(END,txt)
        
 
    
    def set_chain_reg(self,reg,value):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        for id in self.chain.motors.keys():
            self.chain.set_reg(id,reg,value)
        
    def activate(self):
        self.set_chain_reg("torque_enable",1)

    def deactivate(self):
        self.set_chain_reg("torque_enable",0)

appname="DynamixelLab"
root = Tk()
root.title(appname)
mainwindow = MainWindow(root)
root.protocol("WM_DELETE_WINDOW", mainwindow.exit)
root.mainloop()
