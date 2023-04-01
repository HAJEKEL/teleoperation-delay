#Teleoperation-delay

Teleoperation-delay is a test program based on PA1 and PA2. The function of the test program is to test the influence of time delay on a simple 2D path tracking task.

##How to setup
###Delay
The amount of simulated delay can be controlled by changing the length of the Queue. This can be done by changing the variable 'buffer' in the device.py.

###Haply
When using a Haply device make sure you have installed the correct drivers for connecting to an Arduino Zero. When the device is plugged in to the USB port the connection should be made automaticly by running device.py. It is not necessary to use a Haply. When no device can be found the simulation will open a window with a controller.

###Data collection in Excel
At the end of the main.py the test data is printed and collected in an excel file. To set this up you have to make sure the excel file you want to use already exists. Fill in the location filepath of the file and the sheet you want to write to.
When used multiple times keep changing the sheet or the starting row of the writer.

##How to use
Run the files main.py and device.py. When both windows are opened select the main window and press E to start. When using a Haply it should connect automaticly when setup correctly. Otherwise you can use the virtual controller.

When the simulation is started a test can be started by moving the endpoint of the arm to the white starting rectangle.
A test will end when you move the endpoint to the red ending rectangle. 
The goal of the test is to move along the orange line.

