#!/bin/sh

rm ~/PCMSIM_STAGE/pcmsim-fork/objdump_pcmsimko
cd /home/luisky/PCMSIM_STAGE/kernelbuild/buildroot-2019.02.2/output/host/opt/ext-toolchain/bin
./arm-linux-gnueabihf-objdump -D ~/PCMSIM_STAGE/pcmsim-fork/pcmsim.ko >> ~/PCMSIM_STAGE/pcmsim-fork/objdump_pcmsimko

echo "objdump -D of pcmsim.ko into objdump_pcmsimko"

exit 0