# Venus Driver source (Hamster DX) for Linux

__I did not write the driver code__, but I have made some improvements to the makefile and shell script. The original script had early exits and errors that left unnecessary files on the system, the makefile had errors related to the modules directory too. I have fixed these issues and added a few more features to the script.

#### All the rights and credits to Nitgen Co. Ltd. (http://www.nitgen.com)

## Installation
To install the driver, please follow these steps:
```bash
sudo make
sudo sh install-driver.sh
```
