
obj-m := pcmsim.o

pcmsim-y += module.o memory.o pcm.o util.o ramdisk.o

# From: linuxdevcenter.com

# KDIR is the location of the kernel source.  The current standard is
# to link to the associated source tree from the directory containing
# the compiled modules.

#TOOLSDIR := /home/luisky/PCMSIM_STAGE/kernelbuild

KDIR  := $(TOOLSDIR)/linux-2.6.33.7
#KDIR  := /lib/modules/$(shell uname -r)/build

# PWD is the current working directory and the location of our module
# source files.
PWD   := $(shell pwd)

# Not that it's a super useful variable, we could have hardcoded arm in the
# default rule but it's cleaner that way.
#ARCH := arm

# cross compiler and toolchain path EXCEPT WE NEED TO REMOVE the gcc at the end of the the filename,
# the script seems to add it
#XTOOLS := $(TOOLSDIR)/x86_64-gcc-4.8.5-nolibc-aarch64-linux-gnu/gcc-4.8.5-nolibc/aarch64-linux-gnu/bin/aarch64-linux-gnu-

# default is the default make target.  The rule here says to run make
# with a working directory of the directory containing the kernel
# source and compile only the modules in the PWD (local) directory.
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

defconfig:
	$(MAKE) -C $(KDIR) defconfig

# -j5 to make it faster
kernel:
	$(MAKE) -C $(KDIR) -j5

clean:
	rm -rf *.o *.ko *.mod.c Module.* modules.* \
	       *~ .*~ .\#*~ \#*~ .*.cmd .tmp*
