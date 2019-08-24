# PCMSIM

This is a fork of PCMSIM from Peter Macko (<pmacko@eecs.harvard.edu>)SSSS, original
code can be found here: https://github.com/pmacko86/pcmsim license is BSD & GPLv3.

We took the original code and modified it so that it would suit our needs, for
now the part accounting for the differences between read / writes latencies and
the effects of CPU caches is not implemented.

This is a regression but we couldn't find documentation on the code, so instead
of using something we didn't understand completely we removed it altogether.

The difference lies now in how latencies are calculated, for more information
check out the documentation in the wiki !

In order to show the different informations about pcmsim you use cat on /proc/pcmsim

$ sudo insmod pcmsim.ko pcm_capacity_mb=128 pcm_lat_write_coef=25 pcm_lat_read_coef=15

will create a file : /dev/pcm0 of size 128MB with read latencies increased by 50% and write
latencies increased by 150%

---

Still relevant parts of the former README by the original author on PCMSIM:

A block device driver for Linux that simulates the presence of a Phase Change
Memory (PCM) in the system installed in one of the DIMM slots on the
motherboard. The simulator is implemented as a kernel module for Linux that
creates /dev/pcm0 when it is loaded -- a ramdisk-backed block devices with
latencies that of PCM. 

Build the project using the "make" command. Then load the generated kernel
module. This will create /dev/pcm0, which you can then format with your
favorite file system and use for benchmarking. Note that the data are backed
by a RAM disk, which means that all data that you put on the block device
will magically disappear upon reboot or module unload.

Once everything works, load the module. Check dmesg to make sure that the
printed numbers appear reasonable. Then format and mount /dev/pcm0, do your
benchmarks, and umount the disk and unload the module when you are done. Then
check dmesg for a few more statistics.
