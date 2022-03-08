#!/bin/bash

#----------------------------------------------------
#fix auto focus and exposure
#----------------------------------------------------
v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=3

v4l2-ctl -d /dev/video4 --set-ctrl=focus_auto=0
v4l2-ctl -d /dev/video4 --set-ctrl=exposure_auto=3

