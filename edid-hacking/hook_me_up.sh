#!/bin/bash

# Add custom mode for 1920x1080 120Hz
xrandr --newmode "1920x1080_120"  297.00  1920 2008 2052 2200  1080 1084 1089 1125  +HSync +Vsync
xrandr --addmode HDMI-1-3 "1920x1080_120"
xrandr --output HDMI-1-3 --mode "1920x1080_120"

#cvt -r 2560 1440 120
xrandr --newmode "2560x1440_120"  497.76  2560 2608 2640 2720  1440 1443 1448 1525 +hsync -vsync
xrandr --addmode HDMI-1-3 "2560x1440_120"

#cvt 2560 1440 60
xrandr --newmode "2560x1440_60"  312.45  2560 2752 3024 3488  1440 1443 1448 1493 -hsync +vsync
xrandr --addmode HDMI-1-3 "2560x1440_60"

# Select 1920x1080 120Hz
xrandr --output HDMI-1-3 --mode "1920x1080_120"
#xrandr | grep '\*'
