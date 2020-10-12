#!/usr/bin/env python3
import serial, sys
import time, threading, serial

from PyQt5.QtWidgets import * 
from PyQt5 import QtCore, QtGui, QtSvg, uic
from PyQt5.QtGui import * 
from PyQt5.QtCore import * 

from lib.Utils import *

class ArduinoComm(QWidget):
    return_angles = pyqtSignal(str,str,str)
    def __init__(self,console):
        super().__init__()
        self.at_goal = 0

        self.x_lim = [0.,1000.]
        self.y_lim = [0.,1000.]

        self.console = console
        self.arduino_comm = serial.Serial('COM4',57600)

        self.reader = threading.Thread(target=self.read_serial)
        self.reader.start()
    
    def shutdown_connection(self):
        self.arduino_comm.close()

    def read_serial(self):
        log("Serial reader thread started...")
        try:
            while self.arduino_comm.is_open:
                val = self.arduino_comm.readline()
                val = val.decode("utf-8")
                
                log(val)
                if len(val) > 0:
                    if val[0]=='R':
                        # log(val) 
                        tokens = val.split(' ')

                        x = tokens[1][1:]
                        y = tokens[2][1:]
                        d = tokens[3][1:]
                        self.return_angles.emit(x,y,d)
                        
                        self.at_goal = tokens[4][1]
                    else:
                        self.console.addItem(f'{val[:-1]}')
                        # log(val)

                self.console.setCurrentRow(self.console.count()-1)
        except:
            log("Serial reader thread terminated!")
    
    def write_serial(self,pose):
        #TODO: warn user if value is out of range
        log(f'Sent command: {pose}')
        self.arduino_comm.write(str(pose).encode())
        self.at_goal = 0

    def reboot(self):
        cmd = 'REBOOT'
        log(f'Sent command: {cmd}')
        self.arduino_comm.write(str(cmd).encode())
