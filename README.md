# my_led
This is for the project for using LEDs in the Robot-System

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
