# Venus Driver source (Hamster DX) for Linux

__Modifications have been made to the existing driver code__, including improvements to the makefile and shell script, and added support for kernel versions 5.19 and later. The original script contained early exits and errors that left residual files on the system, and the makefile had issues with the modules directory. These issues have been resolved, and additional features have been incorporated into the script. Kernel builds for version 5.19 and later previously encountered errors related to the usb_maxpacket and usb_pipe modules in VenusDrv.c, which have now been rectified.

A bugfix for that occurred on old hardware has also been included, related to swtiolb.

#### All the rights and credits to Nitgen Co. Ltd. (http://www.nitgen.com)

## Installation
To install the driver, please follow these steps:

### Prerequisites
In case its ubuntu or another distro that uses apt natively, install the following packages:
```bash
sudo apt install build-essential linux-headers-$(uname -r)
```

### Build and install
```bash
sudo make
sudo sh install-driver.sh
```
