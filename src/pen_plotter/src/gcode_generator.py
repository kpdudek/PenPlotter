#!/usr/bin/env python3
from math import sin,cos,pi
from Utils import *

class GCodeGenerator(FilePaths):

    def __init__(self):
        super().__init__()
    
    def retract_endpoints(self,filename):
        tmp_file = open(f'{self.user_path}gcode\\tmp','w')
        gcode = open(f'{self.user_path}gcode\\{filename}.nc','r')
        
        lines = gcode.readlines()
        print(f'Lines: {lines}')
        first_line = ''.join(lines[0].split('D')[:-1]) + 'D0\n'
        last_line = ''.join(lines[-1].split('D')[:-1]) + 'D0\n'

        print(first_line)
        tmp_file.write(first_line)
        for line in lines:
            print(line)
            tmp_file.write(line)
        print(last_line)
        tmp_file.write(last_line)
        tmp_file.close()
        gcode.close()

        os.remove(f'{self.user_path}gcode\\{filename}.nc')
        os.rename(f'{self.user_path}gcode\\tmp',f'{self.user_path}gcode\\{filename}.nc')
    
    def polygon(self,n,filename):
        theta = 0.0
        d_theta = (2*pi)/float(n)

        fp = open(f'{self.user_path}gcode\\{filename}.nc','w')
        for idx in range(0,n):
            x = ((cos(theta)*2.5)+3)*57.2957795
            y = ((sin(theta)*2.5)+3)*57.2957795
            fp.write('G1 X%.2f Y%.2f D1\n'%(x,y))
            theta += d_theta
        fp.close()
        # self.retract_endpoints(filename)

def main():
    gcg = GCodeGenerator()
    gcg.polygon(40,'circle_40')

if __name__=='__main__':
    main()