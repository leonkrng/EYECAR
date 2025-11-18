#!/bin/bash

pinctrl set 17 op dl
pinctrl set 4 op dl
i2cset -y 1 0x70 0x00 0x01

while [ true ]; do 
	
	pinctrl set 4 dl
	i2cset -y 1 0x70 0x00 0x01

	rpicam-hello
	
	pinctrl set 4 dh
	i2cset -y 1 0x70 0x00 0x02

	rpicam-hello
done
