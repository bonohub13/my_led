#!/bin/bash
sudo insmod myled.ko
sudo chmod 666 /dev/myled0
alias stop="sudo rmmod myled"
