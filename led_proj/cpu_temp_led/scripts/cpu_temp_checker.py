#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32
from os import system

class WriteLed:
    def __init__(self):
        self.pin17 = '/dev/myled17' # green led
        self.pin23 = '/dev/myled23' # yellow led
        self.pin25 = '/dev/myled25' # red led

    def led_pin17(self, switch): # green led
        if switch == 0 or switch == 1:
#             with open(self.pin17, "w") as led:
#                 led.write(str(switch))
            system('echo {} > {}'.format(switch, self.pin17))
        else:
            raise ValueError('input value for switch must be 1 or 0')

    def led_pin23(self, switch): # yellow led
        if switch == 0 or switch == 1:
#             with open(self.pin23, "w") as led:
#                 led.write(str(switch))
            system('echo {} > {}'.format(switch, self.pin23))
        else:
            raise ValueError('input value for switch must be 1 or 0')

    def led_pin25(self, switch): # red led
        if switch == 0 or switch == 1:
#             with open(self.pin25, "w") as led:
#                 led.write(str(switch))
            system('echo {} > {}'.format(switch, self.pin25))
        else:
            raise ValueError('input value for switch must be 1 or 0')

class CPU_TempChecker(WriteLed):
    def __init__(self, min_temp=40, max_temp=60):
        super().__init__()
        self.min_temp, self.max_temp = min_temp, max_temp
        rospy.Subscriber('/cpu_temp', Float32, self.__cpuTempCB__)
        self.cpuTemp_msg = Float32()

    def __cpuTempCB__(self, msg):
        self.cpuTemp_msg = msg

    def startup(self):
        rate = rospy.Rate(1)
        for i in range(3):
            if i%3 == 0:
                self.led_pin17(1)
            elif i%3 == 1:
                self.led_pin23(1)
            else:
                self.led_pin25(1)
            rate.sleep()
            self.led_pin17(0)
            self.led_pin23(0)
            self.led_pin25(0)
        self.led_pin17(1)
        self.led_pin23(1)
        self.led_pin25(1)
        rate.sleep()

    def run(self):
        counter = 0
        green_led, yellow_led, red_led = 0, 0, 0
        rate_hz = 10
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            if self.cpuTemp_msg.data < self.min_temp:
                green_led = 1
                yello_led, red_led = 0, 0
            elif self.cpuTemp_msg.data < self.max_temp:
                yellow_led = 1
                green_led, red_led = 0, 0
            else:
                red_led = 1
                green_led, yellow_led = 0, 0

            if counter % 10 == 0:
                rospy.loginfo('Temperature of CPU: %s[C]', self.cpuTemp_msg.data)

            self.led_pin17(green_led)
            self.led_pin23(yellow_led)
            self.led_pin25(red_led)
            counter += 1
            rate.sleep()

        self.led_pin17(0)
        self.led_pin23(0)
        self.led_pin25(0)

if __name__ == '__main__':
    rospy.init_node('cpu_temp_checker')
    min_temp = int(input('input min temp[C]: '))
    max_temp = int(input('input max temp[C]: '))
    sub = CPU_TempChecker(min_temp, max_temp)
    sub.startup()
    sub.run()
