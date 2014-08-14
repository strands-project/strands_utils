# USB reset

This little tool allows to reset a node in the USB tree. This is most useful for resetting the Kinect that runs amok once in a while. Run like this:

````
$> lsusb 
Bus 001 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 002 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 003 Device 003: ID 1d27:0601  
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 002 Device 003: ID 04e7:0020 Elo TouchSystems Touchscreen Interface (2700)
Bus 002 Device 004: ID 08bb:29b2 Texas Instruments Japan PCM2902 Audio CODEC
Bus 002 Device 005: ID 05e3:0608 Genesys Logic, Inc. USB-2.0 4-Port HUB
Bus 002 Device 006: ID 0403:6011 Future Technology Devices International, Ltd FT4232H Quad HS USB-UART/FIFO IC
Bus 002 Device 007: ID 046d:c21f Logitech, Inc. F710 Wireless Gamepad [XInput Mode]
Bus 002 Device 008: ID 046d:c52b Logitech, Inc. Unifying Receiver
````

If you want to reset the device itself (here the Xtion, id `1d27:0601`):

````
$> rosrun usbreset usbreset /dev/bus/usb/003/003 
````

To reset the hub itself (that's required quite often):

````
$> rosrun usbreset usbreset /dev/bus/usb/003/001
````
