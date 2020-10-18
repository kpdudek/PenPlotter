#!/usr/bin/env python3

from PyQt5.QtWidgets import * 
from PyQt5 import QtCore, QtGui, QtSvg, uic
from PyQt5.QtGui import * 
from PyQt5.QtCore import * 
import random, sys, os, math, time
import numpy as np
import pathlib
import threading

from Utils.Utils import *
from Utils.PaintUtils import *

import rospy
from pen_plotter.msg import plotter_msg
from pen_plotter.msg import plotter_feedback

class CanvasControl(QWidget,FilePaths,ElementColors,PlotterLogger):
    def __init__(self,fp,console):
        super().__init__()
        self.fp = fp
        self.console = console
        self.init_logging(self.fp,self.console)
        uic.loadUi(f'{self.user_path}ui/canvas_control.ui',self)

    def mousePressEvent(self,e):
        self.window_press = np.array([[e.x()],[e.y()]])
        self.log(f'Window press: ({e.x()},{e.y()})')

class PlotterWindow(QMainWindow,FilePaths,ElementColors,PlotterLogger):
    def __init__(self,screen,fp):
        super().__init__()
        uic.loadUi(f'{self.user_path}ui/main_window.ui',self)

        self.fp = fp
        self.init_logging(self.fp,self.console)

        self.log('Pen plotter launched...',color='g')

        self.connected = False
        self.running_program = False

        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Disconnected.')

        width = self.geometry().width()
        height = self.geometry().height()
        self.move(math.floor((screen.size().width()-width)/2), math.floor((screen.size().height()-height)/2))

        self.state = 'DISCONNECTED'
        self.state_label.setText(self.state)
        
        self.move_to_pose_button.clicked.connect(self.send_setpoint)
        self.zero_button.clicked.connect(self.zero_setpoint)
        self.run_program_button.clicked.connect(self.run_gcode)
        self.set_work_zero_button.clicked.connect(self.set_work_zero)

        self.canvas = CanvasControl(self.fp,self.console)
        self.actionCanvas.triggered.connect(self.launch_canvas)
        self.actionOpen.triggered.connect(self.read_gcode)
        self.actionRun.triggered.connect(self.run_gcode)
        self.actionQuit.triggered.connect(self.exit)
        self.actionConnect.triggered.connect(self.connect)
        self.actionDisconnect.triggered.connect(self.disconnect)
        self.actionReboot.triggered.connect(self.reboot_teensy)

    def connect(self):
        if self.connected:
            self.log('Already connected...')
            return
        
        rospy.init_node('PenPlotterController', anonymous='True')
        self.plotter_cmd = plotter_msg()
        self.plotter_feedback = plotter_feedback()

        self.plotter_cmdr = rospy.Publisher('/plotter_command', plotter_msg, queue_size = 1)
        self.plotter_subscriber = rospy.Subscriber('/plotter_feedback', plotter_feedback, self.update_pose)
        self.write_angle = 0
        self.retract_angle = 10
        self.clearance_angle = 70
        self.connected = True
        self.log('Connected...')
    
    def disconnect(self):
        rospy.signal_shutdown('Terminate Called!')
        self.connected = False
        self.state = 'DISCONNECTED'
        self.state_label.setText(self.state)
        self.log('Disconnected...')

    def update_pose(self,data):
        try:
            self.plotter_feedback = data
            self.x_pose.display(data.x_angle)
            self.y_pose.display(data.y_angle)
            self.d_pose.display(data.servo_angle)
            
            if data.at_goal:
                self.state_label.setText('STOPPED')
            else:
                self.state_label.setText('MOVING')
            
        except:
            self.log('Subscriber call terminated!')
    
    def set_work_zero(self):
        self.plotter_cmd.set_work_zero = 1
        self.plotter_cmdr.publish(self.plotter_cmd)
        self.plotter_cmd.set_work_zero = 0

    def get_servo_angle(self):
        for widget in self.servo_radio_frame.children():
            if isinstance(widget, QRadioButton):
                if widget.isChecked():
                    if widget.text() == 'Draw':
                        draw = self.write_angle
                    elif widget.text() == 'Retract':
                        draw = self.retract_angle
                    elif widget.text() == 'Clearance':
                        draw = self.clearance_angle
        return draw

    def send_setpoint(self):
        if not self.connected:
            self.log('Connect to arduino first!')
            return
        x = self.x_setpoint.value()
        y = self.y_setpoint.value()
        draw = self.get_servo_angle()
        
        self.plotter_cmd.move_type = 'G1'
        self.plotter_cmd.servo_angle = int(draw)
        self.plotter_cmd.setpoint_x_angle = float(x)
        self.plotter_cmd.setpoint_y_angle = float(y)

        self.plotter_cmdr.publish(self.plotter_cmd)

        self.log(f"Sent command: {self.plotter_cmd.move_type} X{self.plotter_cmd.setpoint_x_angle} Y{self.plotter_cmd.setpoint_y_angle} D{self.plotter_cmd.servo_angle}")

    def zero_setpoint(self):
        if not self.connected:
            self.log('Connect to arduino first!')
            return
        draw = self.get_servo_angle()
        
        self.plotter_cmd.move_type = 'G1'
        self.plotter_cmd.servo_angle = int(draw)
        self.plotter_cmd.setpoint_x_angle = 0.0
        self.plotter_cmd.setpoint_y_angle = 0.0

        self.plotter_cmdr.publish(self.plotter_cmd)

        self.log(f"Sent command: {self.plotter_cmd.move_type} X{self.plotter_cmd.setpoint_x_angle} Y{self.plotter_cmd.setpoint_y_angle} D{self.plotter_cmd.servo_angle}")

    def reboot_teensy(self):
        pass

    def exit(self):
        try:
            self.disconnect()
            self.canvas.close()
        except:
            pass
        self.close()

    def launch_canvas(self):
        self.canvas.show()

    def read_gcode(self):
        fname = QFileDialog.getOpenFileName(self, 'Open file',f'{self.user_path}gcode/',"NC files (*.nc)")[0]
        try:
            fp = open(fname,'r')
            self.gcode_file_list.clear()
        except:
            self.log("Couldn't open gcode file!")
            return
        
        for line in fp:
            self.gcode_file_list.addItem(line)

    def assign_gcode_to_setpoints(self,gcode_line):
        tokens = gcode_line.split(' ')
        self.plotter_cmd.move_type = tokens[1]
        self.plotter_cmd.setpoint_x_angle = float(tokens[1][1:])
        self.plotter_cmd.setpoint_y_angle = float(tokens[2][1:])
        d_val = tokens[3][1]
        if d_val == '0':
            self.plotter_cmd.servo_angle = self.retract_angle
        elif d_val == '1':
            self.plotter_cmd.servo_angle = self.write_angle

    def setpoint_reached(self):
        error_x = self.plotter_cmd.setpoint_x_angle - self.plotter_feedback.x_angle
        error_y = self.plotter_cmd.setpoint_y_angle - self.plotter_feedback.y_angle
        if (abs(error_x) < 1.5) and (abs(error_y) < 1.5):
            return True
        else:
            return False
        
    def gcode_worker(self):
        self.running_program = True
        num_commands = self.gcode_file_list.count()

        self.log('Starting to run GCode!')

        for idx in range(0,num_commands):
            gcode_line = self.gcode_file_list.item(idx).text()
            self.assign_gcode_to_setpoints(gcode_line)
            self.gcode_file_list.setCurrentRow(idx)

            self.plotter_cmdr.publish(self.plotter_cmd)

            self.log(f"Sent command: {self.plotter_cmd.move_type} X{self.plotter_cmd.setpoint_x_angle} Y{self.plotter_cmd.setpoint_y_angle} D{self.plotter_cmd.servo_angle}")

            while not self.setpoint_reached():
                pass

        self.log('Done running GCode!')
        self.gcode_file_list.setCurrentRow(0)
        self.running_program = False
    
    def run_gcode(self):
        if not self.connected:
            self.log('Connect to arduino first!')
            return
        elif self.running_program:
            self.log('Program already running!',color='y')
            return
        
        if not self.gcode_file_list.count()==0:
            self.worker = threading.Thread(target=self.gcode_worker)
            self.worker.start()
        else:
            self.log("Load a gcode file first!",color='y')

def main(fp):    
    QApplication.setStyle("fusion")
    dark_mode = True

    app = QApplication(sys.argv)

    # Now use a palette to switch to dark colors
    if dark_mode:
        palette = DarkColors().palette
        app.setPalette(palette)
    else:
        palette = FusionColor().palette
        app.setPalette(palette)
    
    # create the instance of our Window 
    plotter_window = PlotterWindow(app.primaryScreen(),fp) 
    plotter_window.show()

    # start the app 
    sys.exit(app.exec()) 

if __name__ == '__main__':
    fp = open('/var/log/pen_plotter.log','a')
    print('Log file opened...')
    try:
        main(fp)
    finally:
        fp.close()
        print('Log file closed...')
        # self.log('Pen plotter closed...',color='g')