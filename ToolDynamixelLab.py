#!/usr/bin/python
import logging
logging.basicConfig(level=logging.DEBUG)

import os
import time
import json
from string import *
from threading import Thread

from Tkinter import *
import tkMessageBox
import tkSimpleDialog
from serial import SerialException


from dxl import *
from dxl.dxlcore import *

searchRates=[57142,3000000,1000000,9600]






class RosWindow(Thread):

    def __init__(self, master,parent,raw=False):
        Thread.__init__(self)
        self.master=master
        self.parent=parent
        self.chain=parent.chain
        self.raw=raw
        
        self.window=Toplevel(self.master)
        title="ROS SI"
        if raw:
            title="ROS Raw"
        else:
            title="ROS SI"
            
        self.window.title(title)

        self.window.protocol("WM_DELETE_WINDOW", self.destroy)
        self.window.bind('<Key-Escape>', self.destroy )
        
        self.frame=Frame(self.window)
        Button(self.frame,text="Close",command=self.destroy).grid(column=0,row=0)
                
        self.frame.pack()
        self.start()
        
    def run(self):
        import rospy
        from dxl import dxlros
        rospy.init_node("dxl")
        self.dxlros=dxlros.DxlROS(self.chain,rate=100,raw=self.raw)
        rospy.spin()        
        
    def destroy(self):
        import rospy
        self.dxlros.stop()
        #~ rospy.shutdown()        
        self.parent.rosWindow=None
        self.window.destroy()
    


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
        self.localFrame=[]
        for id in self.chain.motors.keys():
            self.generate(id)            
        
        self.frame.pack()
        self.master.after(1000,self.startUpdating) # To avoid spurious set coming from sliders

    def startUpdating(self):
        self.update=True
        
    def generate(self,id): 
        model_name=self.chain.motors[id].model_name
        motor=self.chain.motors[id]
        if motor.is_motor():
            localFrame=LabelFrame(self.frame,text="MOTOR %d %s"%(id,model_name) )
            localFrame.grid(column=self.column,row=self.row)
            self.localFrame.append(localFrame)
            #~ Label(localFrame,text="MOTOR %d %s"%(id,model_name)).grid(column=self.column,row=self.row,columnspan=2)
            #~ self.row+=1
            #~ Separator(self.frame,orient=HORIZONTAL,sticky='ew').grid(column=self.column,row=self.row,columnspan=2)
            #~ self.row+=1
            #~ for rname,reg in motor.registers.items():
                #~ if 'w' in reg.mode and not reg.eeprom:
                    #~ self.addRegister(id,rname)
            
            row=0
            row=self.addRegister(id,localFrame,"goal_pos",row)
            row=self.addRegister(id,localFrame,"moving_speed",row)
            #~ self.addRegister(id,"p_gain")
            self.row+=1
            if self.row>7:
                self.row=0
                self.column+=1

    def addRegister(self,id,localFrame,register,row):
        Label(localFrame,text=register).grid(column=0,row=row)
        val=self.chain.get_reg(id,register)
        reg=self.chain.motors[id].registers[register]
        
        if reg.range:
            range=reg.range
        elif reg.size==1:
            range=[0,255]
        else:
            range=[0,65535]
        
        scale=Scale(localFrame, from_=range[0], to=range[1], length=250,orient=HORIZONTAL)        
        scale.set(val)
        scale.configure( command=lambda val,id=id,register=register: self.set(id,register,val) )
        scale.grid(column=1,row=row)
        return row+1
        
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
        self.rosWindow=None
        
        self.chain=None

        self.width=800
        self.height=600
        self.frame = Frame(master, width= self.width, height= self.height)
        self.buildMenu(master)
        
        self.master.bind('<Key-Escape>', self.exit )

        #~ Label(self.frame, text="WARNING: use with 1 servo connected at a time !!").pack()

        self.portFrame=self.buildSerialPortFrame()
        self.portFrame.grid(column=0,row=0)

        self.listFrame=self.buildListFrame()
        self.listFrame.grid(column=0,row=1)

        self.chainFrame=self.buildChainFrame()
        self.chainFrame.grid(column=1,row=0,rowspan=2)



   
        
        #~ Button(self.frame,text="SyncPos",command=lambda: self.test()).grid(column=11,row=12)
        #~ Button(self.frame,text="SyncSpeed",command=lambda: self.test2()).grid(column=11,row=13)
        
        

        self.frame.pack()


    def buildSerialPortFrame(self):
        frame=LabelFrame(self.frame,text="Serial Port")
        Label(frame,text="Port:").grid(column=0,row=0)
        
        self.comPort= StringVar()
        if os.name=="nt":
            port="COM21"
        else:
            port="/dev/ttyUSB0"
        self.comPort.set(port)
        entryComPort = Entry(frame, textvariable=self.comPort)
        entryComPort.grid(column=1,row=0)        
        Button(frame,text="Scan",command=self.scan).grid(column=2,row=0)

        Label(frame,text="Baudrate:").grid(column=0,row=1)
        self.baudRate= IntVar()
        self.baudRate.set("1000000")
        entryBaudRate= Entry(frame, textvariable=self.baudRate)
        entryBaudRate.grid(column=1,row=1)        
        Button(frame,text="Connect",command=self.connect).grid(column=2,row=1)

        Label(frame,text="Timeout:").grid(column=0,row=2)
        self.timeout= DoubleVar()
        self.timeout.set("0.1")
        entryTimeout= Entry(frame, textvariable=self.timeout)
        entryTimeout.grid(column=1,row=2)        

        return frame

    def buildListFrame(self):
        frame=LabelFrame(self.frame,text="Motor List (use right button)")
        self.listElements = Listbox(frame,width=50,height=20)
        self.listElements.grid(row=0,column=0)
        self.popup = Menu(self.master, tearoff=0)
        self.popup.add_command(label="Change ID",command=self.changeMotorID)
        self.popup.add_command(label="Change baudrate",command=self.changeMotorBaudrate)
        self.popup.add_command(label="Open documentation",command=self.openDocumentation)
        self.listElements.bind("<Button-3>", self.do_popup)        
        return frame

    def buildChainFrame(self):
        frame=LabelFrame(self.frame,text="Motor Chain")

        # Text field for configuration with scrollbars
        configFrame=LabelFrame(frame,text="Chain configuration")
        self.textConfig=Text(configFrame,width=50,height=30)
        self.textConfig.grid(column=0,row=0)
        self.textConfig.insert(END,"{}")
        
        scrly = Scrollbar(configFrame, command=self.textConfig.yview)
        self.textConfig.config(yscrollcommand=scrly.set)
        scrly.grid(column=1,row=0,sticky="ns")
        
        scrlx = Scrollbar(configFrame, command=self.textConfig.xview,orient=HORIZONTAL)
        self.textConfig.config(xscrollcommand=scrlx.set)
        scrlx.grid(column=0,row=1,sticky="ew")
        
        configFrame.grid(column=0,row=0,rowspan=10)
        


        Button(frame,text="Read",command=self.refresh).grid(column=1,row=0)
        Button(frame,text="Write",command=self.set).grid(column=1,row=1)
        Button(frame,text="Activate",command=self.activate).grid(column=1,row=2)
        Button(frame,text="Deactivate",command=self.deactivate).grid(column=1,row=3)

        Button(frame,text="Show Motors",command=lambda: self.createMotorsWindow()).grid(column=1,row=4)
        
        if "--ros" in sys.argv:
            Button(frame,text="Start ROS Raw",command=lambda: self.createRosWindowRaw()).grid(column=1,row=5)
            Button(frame,text="Start ROS SI",command=lambda: self.createRosWindowSI()).grid(column=1,row=6)

        return frame


        
        
    def do_popup(self,event):
        oldid=self.getSelectedMotor()
        if oldid<0:
            return

        # display the popup menu
        try:
            self.popup.tk_popup(event.x_root, event.y_root, 0)
        finally:
            # make sure to release the grab (Tk 8.0a1 only)
            self.popup.grab_release()

    def buildMenu(self, root):
        menubar = Menu(root)
        root.config(menu=menubar)

        filemenu = Menu(menubar)
        menubar.add_cascade(label='File', menu=filemenu)
        #~ filemenu.add_command(label='Load configuration ...', command=self.saveState)
        #~ filemenu.add_command(label='Save configuration ...', command=self.loadState)
        filemenu.add_command(label='Quit', command=self.exit)



        settingsmenu = Menu(menubar)
        menubar.add_cascade(label='Settings', menu=settingsmenu)

        self.doBroadcast=BooleanVar()
        settingsmenu.add_checkbutton(label="Use Ping Broadcast", onvalue=True, offvalue=False, variable=self.doBroadcast)
        self.doBroadcast.set(True)        

        #~ self.doHideInternalStates= BooleanVar()    
        #~ viewmenu.add_checkbutton(label="Hide Internal States", onvalue=True, offvalue=False, variable=self.doHideInternalStates)
        #~ self.doHideInternalStates.set(True)



    def getSelectedMotor(self):
        items = map(int, self.listElements.curselection())    
        if len(items)==0:
            return -1
        else:
            id=self.chain.motors.keys()[items[0]]
            return id
            
        
    def changeMotorID(self):
        oldid=self.getSelectedMotor()
        if oldid<0:
            tkMessageBox.showerror("Selection Error","Please select a motor first")
        else:
            newid=tkSimpleDialog.askinteger("Change ID","Please provide new ID for motor %d"%oldid)
            if newid==None:
                return
            if oldid==newid:
                return
            if newid<1 or newid>Dxl.BROADCAST:
                tkMessageBox.showerror("Range Error","ID should be between 1 and %d"%(Dxl.BROADCAST-1))
            else:
                do=True
                if newid in self.chain.motors.keys():
                    answer=tkMessageBox.askyesno("ID Conflict","Warning: the motor ID %d is already attributed on your chain, are you sure you want to proceed?"%(newid))
                    if not answer:
                        do=False
                if do:
                    self.chain.set_reg(oldid,"id",newid)
                    self.connect()
                
    def changeMotorBaudrate(self):
        id=self.getSelectedMotor()
        if id<0:
            tkMessageBox.showerror("Selection Error","Please select a motor first")
        else:
            rate=tkSimpleDialog.askinteger("Change Baudrate","Please provide the new baudrate for motor ID %d"%id)
            if rate==None:
                return
            reg=self.chain.motors[id].registers["baud_rate"]
            dxlrate=reg.fromsi(rate)
            realrate=reg.tosi(dxlrate)
            
            answer=tkMessageBox.askyesno("Change Baudrate","Warning: motor ID %d will be set to baudrate %d, are you sure you want to proceed?"%(id,realrate))
            if answer:
                self.chain.set_reg(id,"baud_rate",dxlrate)                
                self.connect()

    def openDocumentation(self):
        id=self.getSelectedMotor()
        if id<0:
            tkMessageBox.showerror("Selection Error","Please select a motor first")
        else:
            import webbrowser
            url=self.chain.motors[id].documentation_url
            webbrowser.open(url)

        
    def createMotorsWindow(self):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        if self.motorsWindow==None:
            self.motorsWindow=MotorsWindow(self.master,self)
        
    def createRosWindowRaw(self):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        if self.rosWindow==None:
            self.rosWindow=RosWindow(self.master,self,raw=True)
                
    def createRosWindowSI(self):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        if self.rosWindow==None:
            self.rosWindow=RosWindow(self.master,self,raw=False)
                
    def exit(self,event=None):
        self.close()
        self.master.destroy()

    def open(self,rate):
        self.close()
        comPort=self.comPort.get()
        self.chain=dxlchain.DxlChain(comPort,rate=rate,timeout=self.timeout.get())
        

    def test(self):
        ids=[10,11]
        positions=[200,800]
        self.chain.sync_write_pos(ids,positions)
        
    def test2(self):
        ids=[10,11]
        positions=[200,800]
        speeds=[30,100]
        self.chain.sync_write_pos_speed(ids,positions,speeds)
        
    def close(self):
        if self.motorsWindow:
            self.motorsWindow.destroy()
            self.motorsWindow=None
        if self.rosWindow:
            self.rosWindow.destroy()
            self.rosWindow=None
        if self.chain:
            try:
                self.chain.close()
                self.chain=None
            except:
                loggin.warning("WARNING: could not close chain")
        
    def scan(self):
        selected_rate=None
        self.listElements.delete(0,END)
        for rate in searchRates:            
            try:
                self.open(rate)
            except SerialException,e:
                tkMessageBox.showerror("Serial Error","Could not open serial port: \n"+str(e))
                return
                
            try:
                motors=self.chain.get_motor_list(instantiate=False,broadcast=self.doBroadcast.get())                
                for id in motors:
                    model_number=self.chain.get_model_number(id)
                    model_name=get_model_name(model_number)
                    self.listElements.insert(END, "Rate %d ID %d (%s)\n"%(rate,id,model_name))

                logging.info("rate %d: %s"%(rate,str(motors)))
                if len(motors)>0:
                    selected_rate=rate
                #~ chain.dump()
            except dxlcore.DxlConfigurationException,e:
                tkMessageBox.showerror("Configuration Error","Could not instantiate motor: \n"+str(e))
                return
            except dxlcore.DxlCommunicationException,e:
                tkMessageBox.showerror("Communication Error","Could not communicate with motor (could be due to overlapping IDs, try on single motors): \n"+str(e))
                return
            finally:
                self.close()
        
        if selected_rate:
            self.selectRate(selected_rate,populateList=False)

    def connect(self):
        self.selectRate(self.baudRate.get(),populateList=True)


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
            self.conf=self.chain.get_configuration(broadcast=self.doBroadcast.get())
            self.showConfig(self.conf)
        else:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")


    def selectRate(self,rate,populateList=False):
        if populateList:
            self.listElements.delete(0,END)        
        logging.info("Selected rate %d"%rate)
        self.baudRate.set(rate)
        try:
            self.open(rate)
        except SerialException,e:
            tkMessageBox.showerror("Serial Error","Could not open serial port: \n"+str(e))
            return
        try:            
            self.conf=self.chain.get_configuration(broadcast=self.doBroadcast.get())            
        except dxlcore.DxlConfigurationException,e:            
            tkMessageBox.showerror("Configuration Error","Could not instantiate motor: \n"+str(e))
        
        if populateList:
            for id in self.chain.motors.keys():
                model_name=self.chain.motors[id].model_name
                self.listElements.insert(END, "Rate %d ID %d (%s)\n"%(rate,id,model_name))

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


if "--ros" in sys.argv:
    import rospy
    rospy.init_node("dxl")


root = Tk()
appname="DynamixelLab"
root.title(appname)
root.iconbitmap(default="humarobotics.ico")

mainwindow = MainWindow(root)
root.protocol("WM_DELETE_WINDOW", mainwindow.exit)
root.mainloop()

if "--ros" in sys.argv:
    import rospy
    rospy.signal_shutdown("End of Application")
