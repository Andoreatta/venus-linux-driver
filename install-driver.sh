#!/bin/bash

KER_VER=$(uname -r)
DRIVER_PATH="/lib/modules/$KER_VER/kernel/drivers/usb/misc/"
device="nitgen"
major=180
minor=230

echo "NITGEN USB Finkey HAMSTER-I/DX Driver Install"

if [ ! -d $DRIVER_PATH ]; then
  echo "Driver Path does not exist, creating..."
  mkdir $DRIVER_PATH
fi

if [ -f $DRIVER_PATH/VenusDrv.ko ]; then
  echo "$DRIVER_PATH/VenusDrv.ko already exists"
  echo "Run uninstaller to uninstall driver"
  exit 1
fi

if [ ! -f VenusDrv.ko ] || [ ! -f VenusDrv.h ]; then
  echo "VenusDrv.ko or VenusDrv.h file not found in current directory"
  exit 1
fi

drvkernel=$(/sbin/modinfo VenusDrv.ko | grep vermagic | cut -f8 -d " ")
if [ $drvkernel != $KER_VER ]; then
  echo "Your kernel version $KER_VER does not match with NITGEN USB Driver compiled kernel version $drvkernel"
  read -p "Do you want to proceed anyways? (y/n) " choice
  case "$choice" in
    y|Y ) ;;
    n|N ) exit 1;;
    * ) echo "Invalid choice, exiting..."; exit 1;;
  esac
fi

cp -v VenusDrv.ko $DRIVER_PATH
cp -v VenusDrv.h /usr/include/linux/

# Remove stale node
rm -f /dev/${device}[0-7]

if [ -n "$(mount | grep sysfs)" ] && [ -d "/etc/udev" ]; then
  is_dyn_usb_devfile_use=y
fi

if [ -z "${is_dyn_usb_devfile_use}" ]; then
  for i in {0..7}; do
    mknod /dev/${device}$i c $major $(expr $minor + $i)
    chmod 0666 /dev/${device}$i
  done
else
  cp -av ./99-Nitgen-VenusDrv.rules /etc/udev/rules.d
fi

if [ -f ./shared-object/VenusLib.so ]; then
  cp ./shared-object/VenusLib.so /lib/
else
  echo "File VenusLib.so does not exist in the current directory"
fi

/sbin/insmod $DRIVER_PATH/VenusDrv.ko

/sbin/depmod

echo "NITGEN USB Finkey HAMSTER-I/DX Driver successfully installed"
echo "Disconnect the Device and Plug it back"