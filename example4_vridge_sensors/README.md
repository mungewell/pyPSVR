
To use these scripts under Linux you will need the following packages
```
$ sudo apt-get install python3-setuptools python3-usb python3-zmq python3-numpy python3-hidapi
```

You will also need to clone (download) the following projects and install them:

https://github.com/construct/construct

https://github.com/KieranWynn/pyquaternion

Control of the pyPSVR (for setting VR mode/etc) functions under Windows, however the HID code for reading the sensors does not. The script can be run on seperate (Linux PC) which might be laggy due to network, or in a virtual machine.

Prototype Windows code is now available....

