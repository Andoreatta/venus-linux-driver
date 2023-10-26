#!/bin/sh
KER_VER=$(uname -r)
DRIVER_PATH="/lib/modules/$KER_VER/kernel/drivers/usb/misc/"
DEVICE="nitgen"

echo "Uninstalling driver......."

rm -f /dev/${DEVICE}[0-7]
/sbin/rmmod VenusDrv.ko

[ -f $DRIVER_PATH/VenusDrv.ko ] && rm -f $DRIVER_PATH/VenusDrv.ko
[ -f /usr/include/linux/VenusDrv.h ] && rm -f /usr/include/linux/VenusDrv.h
[ -f /etc/udev/rules.d/99-Nitgen-VenusDrv.rules ] && rm -f /etc/udev/rules.d/99-Nitgen-VenusDrv.rules

/sbin/depmod

echo "Driver uninstalled successfully"