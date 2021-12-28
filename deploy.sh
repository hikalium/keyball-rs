#!/bin/bash -xe
scp $1 pi4:~/target.elf && ssh pi4 -- openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c '"targets rp2040.core0; program target.elf verify reset exit" --log_output log.txt'
