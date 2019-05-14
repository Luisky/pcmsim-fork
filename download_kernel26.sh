#!/bin/sh

# wget download URL
URL=https://mirrors.edge.kernel.org/pub/linux/kernel/v2.6/linux-2.6.33.7.tar.xz

# wget log file
LOGFILE=wget.log

# download
wget $URL -o $LOGFILE