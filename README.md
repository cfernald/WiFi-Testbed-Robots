# WiFi-Testbed-Robots
The code for the Ros implementation of the robotic chassis control
The code here is all from the University of Texas at Austin Pharos Lab in the Electrical and Computer Engineering department

To get any scripts to work 
make sure you cloned onto your home directory!!!

# Python Files
Circlegnerator and Squaregenerator are both python files taht will generate the GPS coordinates in the shape of a square or circle givien the center GPS coordinates and the radius or apex of the circle, or square respectvely. 

#Setup Script
The set up script will set the entire ROS direcotory and all of the other commands that you need to do in order to download and install ROS onto a linux machine. It will also run scripts to make sure that the catkin files are downloaded correctly.

#testing Script
This script does the same ting as the setip scipt minus all of the downloading.

#in the autobots directory 

##CompassV2
This is just modified compass code from the SparkFun Library the only added things are the fact that you can now change the address of some of the compasses and there is an alternative way to get the pitch, heading, and roll. The old Pharos code has this as well, but the sparkfun one is easier to read. Note: you need to put this in your arduino library in order to use this.
##bin
###gpsPort
This will set up the udev rules for the GPS usb so that you can use the ros (it's name gets set to /dev/gps) this is because the arudino pro mini uses the same exat FTDI chip and their names can get confued. To use this ONLY have the gps plugged in. Run the script. Then restart the computer

####indigo
running source indigo on your machine will do everything you need to recompile the code in your catkin_ws (it does the same thing as catkin_make and source devle/setup.bash) You don't need to change your directoy or anything just source indigo and it will be compiled for you

###motorPort
Does almost the same thing as the gpsPort except for the motor contoller (the arduino pro mini) it names the port to /dev/motor/. The instructions are the same as the gps

###internet.sh
This will set up the WiFi card by running this script. For more insructions on this driver go to https://github.com/porjo/mt7601

###wtfWifi
In case you update your linux machine the wifi card may break so run the wtfWifi script and it will restart it and fix it

##Catkin_ws
The main commands you need to know:
all of the testing roslaunch files
This will just run the components so that you can test and see if that compenent is working

This will test and amke sure the motor and servo is running correctly
roslaunch traxxas_node test_drive.launch

This will amke sure that the compass is outputting data
roslaunch proteus3_gps_hydro test_compass.launch

this will amke sure taht the gos is outputting data
roslaunch proteus3_compass_hydro test_gps.launch

to actually run things 
roslaunch outdoor_navigation navigate_single.launch

