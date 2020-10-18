#!/usr/bin/env python3

import os, sys, time
import datetime as dt
from threading import Thread
import inspect, pathlib

class FilePaths():
    if sys.platform == 'win32':
        user_path = str(pathlib.Path().absolute()) + '\\'
        lib_path = user_path + 'lib\\'
        cc_lib_path = 'cc_lib.dll'
    elif sys.platform == 'linux':
        # print(__file__)
        tokens = str(__file__).split('/')
        # print(tokens)
        # print('/'.join(tokens[:-2]))
        user_path = '/'.join(tokens[:-2])+'/'
        # user_path = str(pathlib.Path().absolute()) + '/'
        lib_path = user_path + 'lib/'
        cc_lib_path = 'cc_lib.so'
    else:
        raise Error('OS not recognized!')

class PlotterLogger():
    def __init__(self):
        super().__init__()
        self.fp = None
        self.console = None
    
    def init_logging(self,fp,console):
        self.fp = fp
        self.console = console
        # self.log('Initializing Logger...')
    
    def log(self,text, color=None):
        '''
        Display the text passed and append to the logs.txt file
        parameters:
            text (str): Message to be printed and logged
            color (str): Optional. Color to print the message in. Default is white.
        '''
        if self.fp == None:
            print('Tried to log without a file pointer!')
            return
        RESET = '\033[m' # reset to the default color
        GREEN =  '\033[32m'
        RED = '\033[31m'
        YELLOW = '\033[33m'
        CYAN = '\033[36m'

        BOLD = '\033[1m'
        UNDERLINE = '\033[2m'

        # Strip newline characters
        text = text.strip()

        # Prepare log message's time of call and filename that the function is called in
        curr_time = '[%s]'%(str(dt.datetime.now())) # date and time

        frame = inspect.stack()[1]
        filepath = frame[0].f_code.co_filename
        if sys.platform == 'win32':
            filename = ' (%s)'%(filepath.split('\\')[-1].split('.')[0])
        elif sys.platform == 'linux':    
            filename = ' (%s)'%(filepath.split('/')[-1].split('.')[0])
        else:
            filename = ''

        # Form log message
        log_msg = curr_time + filename + ' ' + text

        # Print to terminal in specified color
        if sys.platform == 'win32':
            # print(log_msg)
            if color == 'g' or color == 'G':
                print(GREEN + log_msg + RESET)
            elif color == 'r' or color == 'R':
                print(RED + log_msg + RESET)
            elif color == 'y' or color == 'Y':
                print(YELLOW + log_msg + RESET)
            elif color == 'c' or color == 'C':
                print(CYAN + log_msg + RESET)
            else:
                print(log_msg)
        elif sys.platform == 'linux':    
            if color == 'g' or color == 'G':
                print(GREEN + log_msg + RESET)
            elif color == 'r' or color == 'R':
                print(RED + log_msg + RESET)
            elif color == 'y' or color == 'Y':
                print(YELLOW + log_msg + RESET)
            elif color == 'c' or color == 'C':
                print(CYAN + log_msg + RESET)
            else:
                print(log_msg)
        else:
            print(log_msg)

        if text == 'Pen plotter launched...':
            self.fp.write('\n\n'+ log_msg+'\n')
        else:
            self.fp.write(log_msg+'\n')

        self.console.addItem(log_msg)
        self.console.setCurrentRow(self.console.count()-1)

def hide_all_widgets(layout):
    items = (layout.itemAt(i) for i in range(layout.count())) 
    for item in items:
        item = item.widget()
        try:
            item.hide()
        except:
            pass
