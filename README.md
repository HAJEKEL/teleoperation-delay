## Introduction

Teleoperation is the process of controlling a robotic system remotely, typically through a human-machine interface. Time delay is an inherent issue in teleoperation, as the communication between the operator and the robot takes time, and this delay can affect the performance of the system. In this paper, we present a test program, called Teleoperation-delay, that aims to investigate the influence of time delay on a simple 2D path tracking task.

## Methods

The Teleoperation-delay program is based on the PA1 and PA2 frameworks. The test program allows the user to control a robotic arm and track a 2D path using either a Haply device or a virtual controller. The user can adjust the amount of simulated delay by changing the length of the Queue, which is controlled by modifying the `buffer` variable in `device.py`.

At the end of each test, the program prints and collects test data in an Excel file. The user needs to specify the location filepath of the file and the sheet to write to. Multiple tests can be run, and the user needs to change the sheet or the starting row of the writer to avoid overwriting previous data.

## Usage

To use the Teleoperation-delay program, the user needs to run the files `main.py` and `device.py`. Once both windows are open, the user needs to select the main window and press "E" to start the simulation. If a Haply device is used, it should connect automatically if the correct drivers for connecting to an Arduino Zero are installed. Otherwise, the user can use the virtual controller.

Once the simulation has started, the user can initiate a test by moving the endpoint of the arm to the white starting rectangle. The test ends when the endpoint is moved to the red ending rectangle, and the goal is to move along the orange line.

## Conclusion

Teleoperation-delay is a test program that provides a platform to investigate the effects of time delay on a simple 2D path tracking task. The program allows the user to control a robotic arm using either a Haply device or a virtual controller, and adjust the amount of simulated delay. The test data is collected in an Excel file, and the program can run multiple tests. The program can be used to explore the impact of time delay on teleoperation performance and inform the design of teleoperation systems.
