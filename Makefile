obj-m := pcmsim.o

pcmsim-y += module.o memory.o pcm.o util.o ramdisk.o

# From: linuxdevcenter.com

# KDIR is the location of the kernel source.  The current standard is
# to link to the associated source tree from the directory containing
# the compiled modules.
KDIR  := /lib/modules/$(shell uname -r)/build

# Buildroot output path (change this to suit your configuration)
BR_OUTPUT := /home/luisky/PCMSIM_STAGE/kernelbuild/buildroot-2019.02.2/output/

# Buildroot Kernel source
BR_KDIR := $(BR_OUTPUT)build/linux-4.19.36

# Cross compiler location
BR_CROSS := $(BR_OUTPUT)host/usr/bin/arm-linux-gnueabihf-

# Flags to compile the module
# -march=armv7-a needed for armv7 instructions
# the rest is for NEON instructions
#KCFLAGS := "KCFLAGS=-mcpu=cortex-a8 -march=armv7-a -mfpu=neon -mfloat-abi=softfp"
KCFLAGS := "KCFLAGS=-mcpu=cortex-a8 -march=armv7-a"

# PWD is the current working directory and the location of our module
# source files.
PWD   := $(shell pwd)

# default is the default make target.  The rule here says to run make
# with a working directory of the directory containing the kernel
# source and compile only the modules in the PWD (local) directory.
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

.PHONY: arm_export
arm_export: arm export

arm:
	$(MAKE) $(KCFLAGS) ARCH=arm CROSS_COMPILE=$(BR_CROSS) -C $(BR_KDIR) M=$(PWD) modules

export:
	sudo ./export.sh

.PHONY: objdump_pcm
objdump_pcm:
	./objdump_pcm.sh

.PHONY: clean
clean:
	rm -rf *.o *.ko *.mod.c Module.* modules.* \
	       *~ .*~ .\#*~ \#*~ .*.cmd .tmp* *.*.dwo .*.dwo objdump_pcmsimko
