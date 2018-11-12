# pyPSVR
Python scripts for control and interaction with the Playstation PSVR

Supports both Linux and Windows, with Python 3 or 2 (using 'six' module).

Windows requires 'libusb' to be installed. In order for the sensors to also
be readably the libusb driver should be applied to the 'Endpoint 5' (NOT 
the 'Composite Device', as instructed previously).

Example usage:
```
D:\>python pyPSVR.py --help
usage: pyPSVR.py [-h] [-S] [-o] [-O] [-v] [-c] [-l] [-L LEDVAL] [-N LEDNUM]
                 [-s SIZE] [-d DIST] [-m MIST] [-b BRIGHT] [-H HDMI] [-k] [-r]
                 [-i] [-C] [-G REG] [-R]

optional arguments:
  -h, --help            show this help message and exit
  -S, --shutdown        Shutdown the PSVR
  -o, --on              Turn the PSVR on
  -O, --off             Turn the PSVR off
  -v, --vrmode          Turn on VR mode
  -c, --cinemode        Turn on Cinematic mode
  -l, --leds            Turn on the LEDs
  -L LEDVAL, --ledval LEDVAL
                        Set brightness values for the LEDs
  -N LEDNUM, --lednum LEDNUM
                        Set brightness only for specific LED
  -s SIZE, --size SIZE  Set the size of the Cinematic screen
  -d DIST, --dist DIST  Set the distance of the Cinematic screen
  -m MIST, --mist MIST  Set the mistery of the Cinematic screen
  -b BRIGHT, --bright BRIGHT
                        Set the brightness of the screen
  -H HDMI, --hdmi HDMI  Set the HDMI resolution of Social-Screen
  -k, --lock            Lock the Cinematic screen in position
  -r, --recenter        Recenter the screen in Cinematic Mode
  -i, --id              Request serial and revision
  -C, --cal             Request calibration data
  -G REG, --reg REG     Request register data
  -R, --read            After writing command, listen for responses
```

Switching to VR mode with the LEDs enabled (on Linux):
```
$ sudo python pyPSVR.py -v -l
Detaching: 5
Re-attaching: 5
```
