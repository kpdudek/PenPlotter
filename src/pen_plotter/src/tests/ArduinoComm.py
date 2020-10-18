#!/usr/bin/env python3
import serial
import sys
from concurrent.futures import ThreadPoolExecutor
import time

def read_serial(arduino_comm):
    while arduino_comm.is_open:
        val = arduino_comm.readline()
        print(val.decode("utf-8"))

def write_serial(arduino_comm):
    while True:
        # print('Enter X setpoint coordinate: ')
        try:
            user_input = input('Enter X setpoint coordinate: ')
            try:
                #TODO: warn user if value is out of range
                f_in = float(user_input)
                # if (f_in >= 0.) and (f_in <= 1000.):
                arduino_comm.write(user_input.encode())
                    # time.sleep(3.0)
                # else:
                #     print('Value outside axis limit!')
            except:
                print("Couldn't convert input to float!")
        except KeyboardInterrupt:
            return

def main(arduino_comm):
    executor = ThreadPoolExecutor(4)
    future = executor.submit(read_serial,arduino_comm)
    writer = executor.submit(write_serial,arduino_comm)
    while not future.done():
        pass

if __name__ == '__main__':
    try:
        arduino_comm = serial.Serial('COM4',9600)
        main(arduino_comm)

    finally:
        arduino_comm.close()
        print('Arduino socket closed!')