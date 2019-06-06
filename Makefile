obj-m := pcmsim.o

pcmsim-y += module.o memory.o pcm.o util.o ramdisk.o

# From: linuxdevcenter.com

# KDIR is the location of the kernel source.  The current standard is
# to link to the associated source tree from the directory containing
# the compiled modules.
KDIR  := /lib/modules/$(shell uname -r)/build

# Added as a test
BUILDROOT_KDIR := /home/luisky/PCMSIM_STAGE/kernelbuild/buildroot-2019.02.2/output/build/linux-4.19.36

# Added as a test
CROSS := /home/luisky/PCMSIM_STAGE/kernelbuild/buildroot-2019.02.2/output/host/usr/bin/arm-linux-gnueabihf-

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
	$(MAKE) KCFLAGS=-march=armv7-a ARCH=arm CROSS_COMPILE=$(CROSS) -C $(BUILDROOT_KDIR) M=$(PWD) modules

export:
	sudo ./export.sh

.PHONY: objdump_pcm
objdump_pcmsimko:
	./objdump_pcm.sh

.PHONY: clean
clean:
	rm -rf *.o *.ko *.mod.c Module.* modules.* \
	       *~ .*~ .\#*~ \#*~ .*.cmd .tmp* *.*.dwo .*.dwo objdump_pcmsimko
