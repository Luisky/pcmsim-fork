#!/bin/sh

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

rm /run/media/luisky/rootfs/root/pcmsim.ko
cp pcmsim.ko /run/media/luisky/rootfs/root/pcmsim.ko
umount /dev/sdb1 /dev/sdb2

if [ $? = 1 ]
then
        echo "error: is SD card plugged in ?"
        exit 1
fi

echo "pcmsim.ko updated on SD card and /dev/sdb1 & /dev/sdb2 unmounted"

exit 0