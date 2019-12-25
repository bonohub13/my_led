# my_led
This is for the project for using LEDs in the Robot-System

## What this does
Luminates the LED by the CPU temperature by 3 levels.
You can change the maximum and the minimal temperature (Celsius) to change when the LED luminate.
By default, it is 40 degrees Celsius for the minimal temperature and 60 degrees Celsius for the maximum temperature.

## What you'll need
- ROS environment (The guy who made this did it in ROS Melodic)
- Raspberry Pi (better with ubuntu) If you are using Raspbian, you will need to modify the kernel
- bread board, 3 LED with different colors, and some resistors (around 200 Ohms)

## Instructions
1. compile three device driver files (pin17/myled_17.c, pin23/myled_23.c, pin25/myled_25.c)
2. active the modules and give them permission of access
    - ```$sudo insmod myled_**.ko; sudo chmod 666 /dev/myled*```

3. add the ROS packages to src/ directory under a workspace
4. compile with ```catkin_make```
5. run the cpu_temp_led on a Raspberry Pi with 3 LEDs
6. run the raspi_cpu_temp on any device with Linux installed

## Video
URL
- https://www.youtube.com/watch?v=26f_r5yqvjk

## Updates
December 25, 2019</br>
- Integrated three of the device drivers for the LEDs into one script
	- Leaving the original three, (pin17, pin23, pin25) for stability (these three has been tested, and is stable)
