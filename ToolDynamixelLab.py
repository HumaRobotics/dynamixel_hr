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
import tkFileDialog


from serial import SerialException


from dxl import *
from dxl.dxlcore import *
from python_text import *

searchRates=[57142,3000000,1000000,9600]

def frate(data):
    if data>=250:
        if data==250: return 2250000
        if data==251: return 2500000
        if data==252: return 3000000
        return 1000000
    else:
        return 2000000/(data+1)
    
        
searchAllRates=[ frate(data) for data in range(0,253)]






class Evaluator:
    def __init__(self):
        self.symbols={}
        
    def bindSymbol(self,name,var):
        self.symbols[name]=var
        
    def perform(self,toeval):
        if toeval.strip()=="": return None
        
        # Wrap in function with return value
        self.cmd="def _localfunction():\n"
        for l in toeval.split("\n"):
            self.cmd+="  "+l+"\n"
        self.cmd+="_return=_localfunction()"
        
        # Build context        
        context=self.buildContext()
        #~ for k,v in context.items():
            #~ print k+" "+str(v)
        
        # Execute and retrieve return value
        exec self.cmd in context
        return context["_return"]
    
    def buildContext(self):
        # Build context        
        context=dict(self.symbols)
        return context
        

class PythonWindow:

    def __init__(self, master,parent):
        self.master=master
        self.parent=parent
        self.chain=parent.chain
        
        self.defaultCode="""
# Use the 'chain' object to access motors
# Here is an example that assumes a motor on ID 1

id=1

chain.goto(id,0,speed=0) # Full speed to pos 0
chain.goto(id,1000,speed=100) # Low speed to pos 1000
chain.goto(id,500,blocking=False) # Current speed to pos 500
while chain.is_moving():
	print chain.get_reg_si(id,"present_position")
chain.goto(id,100,speed=0) # Full speed back to pos 100   
        """
        
        self.window=Toplevel(self.master)
        self.window.title("Python sandbox") 

        self.window.protocol("WM_DELETE_WINDOW", self.destroy)
        self.window.bind('<Key-Escape>', self.destroy )
        
        self.frame=Frame(self.window, width= 300, height= 200)
        self.buildMenu(self.window)

        self.pythonFrame=LabelFrame(self.frame,text="Python code")
        self.textTask=PythonText(self.pythonFrame,width=60,height=30,maxundo=1000,undo=1)
        self.textTask.pack()
        self.pythonFrame.grid(row=0,column=0)
        self.textTask.insert(END,self.defaultCode)
        
        self.evaluator=Evaluator()
        self.evaluator.bindSymbol("chain",self.chain)
        self.textTask.colorize()
        
        scrly = Scrollbar(self.frame, command=self.textTask.yview)
        self.textTask.config(yscrollcommand=scrly.set)
        scrly.grid(column=1,row=0,sticky="ns")
        
        #~ scrlx = Scrollbar(self.frame, command=self.textTask.xview,orient=HORIZONTAL)
        #~ self.textTask.config(xscrollcommand=scrlx.set)
        #~ scrlx.grid(column=0,row=1,sticky="ew")

        Button(self.frame,text="Execute",command=self.execute).grid(column=0,row=1)
        self.textTask.bind("<F5>",self.execute)


        self.frame.pack()
        

    def buildMenu(self, root):
        menubar = Menu(root)
        root.config(menu=menubar)

        filemenu = Menu(menubar)
        menubar.add_cascade(label='File', menu=filemenu)

        filemenu.add_command(label='Save...', command=self.save)
        filemenu.add_command(label='Load...', command=self.load)


    def destroy(self,event=None):
        self.parent.pythonWindow=None
        self.window.destroy()
        
    
    def execute(self,event=None):
        try:
            toeval=self.textTask.get(1.0,END)
            self.evaluator.perform(toeval)
        except Exception,e:
            tkMessageBox.showerror("Python Error",str(e))
            
        
        
    def save(self):
        options={}
        options['defaultextension'] = '.py'
        options['filetypes'] = [('Python script', '.py'),('all files', '.*')]
        file=tkFileDialog.asksaveasfilename(**options)
        if file:
            txt=self.textTask.get(1.0,END)
            f=open(file,"w")
            f.write(txt)
            f.close()
    
    def load(self):
        options={}
        options['defaultextension'] = '.py'
        options['filetypes'] = [('Python script', '.py'),('all files', '.*')]
        file=tkFileDialog.askopenfilename(**options)
        if file:
            self.textTask.delete(1.0,END)
            f=open(file,"r")
            txt=f.read()
            f.close()
            self.textTask.insert(END,txt)
            self.textTask.colorize()
    



class MotorsWindow:

    def __init__(self, master,parent):
        self.master=master
        self.parent=parent
        self.chain=parent.chain
        
        self.window=Toplevel(self.master)
        self.window.title("Motors")        

        self.window.protocol("WM_DELETE_WINDOW", self.destroy)
        self.window.bind('<Key-Escape>', lambda event: self.destroy() )
        
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
        self.pythonWindow=None
        
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
            port="/dev/ttyACM0"
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
        self.listElements = Listbox(frame,width=30,height=20)
        self.listElements.grid(row=0,column=0)
        self.popup = Menu(self.master, tearoff=0)
        self.popup.add_command(label="Enable",command=self.enableMotor)
        self.popup.add_command(label="Disable",command=self.disableMotor)
        self.popup.add_command(label="Change ID",command=self.changeMotorID)
        self.popup.add_command(label="Change baudrate",command=self.changeMotorBaudrate)
        self.popup.add_command(label="Factory reset",command=self.factoryReset)
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
        Button(frame,text="Enable",command=self.activate).grid(column=1,row=2)
        Button(frame,text="Disable",command=self.deactivate).grid(column=1,row=3)

        Button(frame,text="Show Motors",command=lambda: self.createMotorsWindow()).grid(column=1,row=4)
        Button(frame,text="Show Python",command=lambda: self.createPythonWindow()).grid(column=1,row=5)

        Button(frame,text="Save Pose",command=self.savePose).grid(column=1,row=6)
        Button(frame,text="Load Pose",command=self.loadPose).grid(column=1,row=7)
        

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


        self.doScanAll=BooleanVar()
        settingsmenu.add_checkbutton(label="Scan All Rates", onvalue=True, offvalue=False, variable=self.doScanAll)
        self.doScanAll.set(False)        

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

    def disableMotor(self):
        id=self.getSelectedMotor()
        if id<0:
            tkMessageBox.showerror("Selection Error","Please select a motor first")
        else:
            self.chain.disable(id)

    def enableMotor(self):
        id=self.getSelectedMotor()
        if id<0:
            tkMessageBox.showerror("Selection Error","Please select a motor first")
        else:
            self.chain.enable(id)

    def factoryReset(self):
        id=self.getSelectedMotor()
        if id<0:
            tkMessageBox.showerror("Selection Error","Please select a motor first")
        else:
            answer=tkMessageBox.askyesno("Factory Reset","Warning: you are about to completely reset motor ID %d, its ID and baudrate will be changed, are you sure you want to proceed?"%(id))
            if not answer: return
            do=True
            if 1 in self.chain.motors.keys():
                answer=tkMessageBox.askyesno("ID Conflict","Warning: the motor ID 1 obtained after factory reset is already attributed on your chain, are you sure you want to proceed?")
                if not answer:
                    do=False
            if do:
                self.chain.factory_reset(id)
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

    def createPythonWindow(self):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        if self.pythonWindow==None:
            self.pythonWindow=PythonWindow(self.master,self)
        #~ from idlelib.PyShell import EditorWindow
        #~ self.pythonWindow=EditorWindow(root=self.master)
        
                
                
    def exit(self,event=None):
        self.close()
        self.master.destroy()

    def open(self,rate):
        self.close()
        comPort=self.comPort.get()
        if not self.chain:
            self.chain=dxlchain.DxlChain(comPort,rate=rate,timeout=self.timeout.get())
        else:
            self.chain.reopen(portname=comPort,rate=rate,timeout=self.timeout.get())
        

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
        #~ if self.pythonWindow:
            #~ self.pythonWindow.destroy()
            #~ self.pythonWindow=None
        if self.chain:
            try:
                self.chain.close()
                #~ self.chain=None
            except:
                loggin.warning("WARNING: could not close chain")
        
    def scan(self):
        selected_rate=None
        self.listElements.delete(0,END)
        rates=searchRates
        if self.doScanAll.get():
            rates=searchAllRates
        for rate in rates:            
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
                    self.listElements.insert(END, "Rate %d ID %d (%s)"%(rate,id,model_name))

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
        except dxlcore.DxlCommunicationException,e:
            tkMessageBox.showerror("Communication Error","Could not communicate with motor (could be due to overlapping IDs, try on single motors): \n"+str(e))
            return
        
        if populateList:
            for id in self.chain.motors.keys():
                model_name=self.chain.motors[id].model_name
                self.listElements.insert(END, "Rate %d ID %d (%s)"%(rate,id,model_name))

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
    
    def savePose(self):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        options={}
        options['defaultextension'] = '.position'
        options['filetypes'] = [('pose files', '.position'),('all files', '.*')]
        file=tkFileDialog.asksaveasfilename(**options)
        if file:                        
            self.chain.save_position(file)

    def loadPose(self):
        if not self.chain:
            tkMessageBox.showerror("Chain Error","Please connect to a valid chain first")
            return
        options={}
        options['defaultextension'] = '.position'
        options['filetypes'] = [('pose files', '.position'),('all files', '.*')]
        file=tkFileDialog.askopenfilename(**options)
        if file:
            self.chain.load_position(file)

        



root = Tk()
appname="DynamixelLab"
root.title(appname)
try:
    root.iconbitmap(default="humarobotics.ico")
except:
    logging.warning("Could not load icon")
    

mainwindow = MainWindow(root)
root.protocol("WM_DELETE_WINDOW", mainwindow.exit)
root.mainloop()

