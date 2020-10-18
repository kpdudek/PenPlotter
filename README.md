# PenPlotter
The hardware can be controlled with any microcontroller compatible with the Arduino IDE. However, the processor MUST have a clock speed of 180MHz or greater, and have at least 4 interrupt pins.

Given these constraints, the Teensy 3.6 was selected.

## Install
Install the arduino IDE from here: https://www.arduino.cc/en/Main/Software

To use the Teensy with the Arduino IDE, install teensyduino following these instructions: https://www.pjrc.com/teensy/teensyduino.html

Install python 3 and pip:
```
sudo apt-get install python3
sudo apt-get install python3-pip
```

Clone the repository:
```
sudo apt-get install git
git clone https://github.com/kpdudek/PenPlotter.git
```

Install PyQt5 as the GUI framework:
```
pip3 install PyQt5 
```

The communication from the GUI to the microcontroller is handled using ROS. Download the full ROS desktop install following these instructions: http://wiki.ros.org/melodic/Installation/Ubuntu

Then install rosserial for arduino:
```
sudo apt-get install python-catkin-tools
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial
```

Install numpy:
```
pip3 install numpy
```

### Logging
The expected log file is `/var/log/pen_plotter.log`. Rigt now there is no installer, so you must create this file manually:
```
sudo touch /var/log/pen_plotter.log
sudo chmod 777 /var/log/pen_plotter.log
```

### Uploading Arduino Sketch
Copy the `ros_lib/` folder from the cloned directory to your `Arduiono/libraries/` folder whereever that may be. It is likely located in your `home/` folder.

Upload the sketch as normal.


## Usage
### To launch the Pen Plotter GUI
```
rosrun pen_plotter PenPlotter.py
```

### To connect to the Teensy Microcontroller
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyS# _baud:=57600
```
substitute the `#` in `ttyS#` for the `COM` port number listed in the Arduino IDE, and use the appropriate baud rate.