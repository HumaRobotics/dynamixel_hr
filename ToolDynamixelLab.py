#!/usr/bin/python
import logging
logging.basicConfig(level=logging.DEBUG)

import time
import json
from string import *

from Tkinter import *
import tkMessageBox

from dxl import *

searchRates=[57142,3000000,1000000,9600]


class MainWindow:

    def __init__(self, master):
        self.master=master

        self.width=800
        self.height=600
        self.frame = Frame(master, width= self.width, height= self.height)
        self.master.bind('<Key-Escape>', self.exit )

        #~ Label(self.frame, text="WARNING: use with 1 servo connected at a time !!").pack()

        Label(self.frame,text="Port:").grid(column=0,row=0)
        
        self.comPort= StringVar()
        self.comPort.set("COM21")
        entryComPort = Entry(self.frame, textvariable=self.comPort)
        entryComPort.grid(column=1,row=0)
        
        Button(self.frame,text="Scan",command=self.scan).grid(column=2,row=0)

        Label(self.frame,text="Baudrate:").grid(column=3,row=0)
        self.baudRate= IntVar()
        self.baudRate.set("3000000")
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

        self.chain=None
        self.frame.pack()
        

    def exit(self,event=None):
        self.close()
        self.master.destroy()

    def open(self,rate):
        self.close()
        comPort=self.comPort.get()
        self.chain=dxlchain.DxlChain(comPort,rate=rate)

    def close(self):        
        if self.chain:
            try:
                self.chain.close()
                self.chain=None
            except:
                print "WARNING: could not close chain"
        
    def scan(self):
        selected_rate=None
        for rate in searchRates:
            self.open(rate)
            try:
                self.chain.get_motor_list()                
                print "rate %d: %s"%(rate,self.chain.motors.keys())
                if len(self.chain.motors.keys())>0:
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
            self.chain.dump()
            self.showConfig(self.conf)
        else:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")


    def selectRate(self,rate):
        print "Selected rate %d"%rate
        self.baudRate.set(rate)
        self.open(rate)
        self.conf=self.chain.get_configuration()
        self.chain.dump()
        self.showConfig(self.conf)

    def showConfig(self,config):
        txt=json.dumps(config,indent=4,sort_keys=False)
        self.textConfig.delete(1.0,END)            
        self.textConfig.insert(END,txt)
        
    def bench(self):
        comPort=self.comPort.get()
        servo=atoi(self.servoId.get())
        print "Connect with maximum rate at id %d"%servo
        actuator = ServoController(comPort,baudRate=3000000)
        benchGetPosition(actuator,servo)
        actuator.Close()

    def getRegisters(self):
        comPort=self.comPort.get()
        servo=atoi(self.servoId.get())
        print "Connect with maximum rate at id %d"%servo
        actuator = ServoController(comPort,baudRate=3000000)
        regs=actuator.GetAllReg(servo)
        print "Motor %d, regs %s\n"%(servo,str(regs))
        actuator.Close()

    def factoryReset(self):
        comPort=self.comPort.get()
        print "Locating servo"
        res=identifyServo(comPort)
        if res==None:
            print "ERROR: Could not find any servo"
            exit()
        print "Servo found: "+str(res)
        rate=res[0]
        oldId=res[1]

        print "Performing Factory Reset"
        actuator = ServoController(comPort,baudRate=rate)
        actuator.FullReset()
        actuator.Close()
        print "Done"
    
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

appname="DxlLab"
root = Tk()
root.title(appname)
mainwindow = MainWindow(root)
root.protocol("WM_DELETE_WINDOW", mainwindow.exit)
root.mainloop()
