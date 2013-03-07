#!/bin/bash
unbuffer python joystick_in.py 0| unbuffer -p nxjpc -cp $CLASSPATH se.ajn.slam.pchost.PCBridge 0| python nxt_slam.py
#unbuffer python joystick_in.py 0| nxjpc -cp $CLASSPATH se.ajn.slam.pchost.PCBridge > log.txt
#unbuffer python joystick_in.py
#nxjpc -cp $CLASSPATH se.ajn.slam.pchost.PCBridge
