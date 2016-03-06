#!/bin/bash
#
# Copyright (c) 2016 CliveLiu.
# All rights reserved.
#

function error ()
{
    echo -e "\033[31mERROR: \033[0m"$1
    exit 1
}

# Make sure that current directory is linux's path.
CUR_PATH=`pwd`
CUR_DIR=${CUR_PATH##*/}
[ "${CUR_DIR}" != "linux-xlnx" ] && error "Current directory isn't linux's path."

# Check environment variables
[ -z ${ARCH} ] && error "Please perform \"$ source setup_env.sh\"."
[ -z ${CROSS_COMPILE} ] && error "Please perform \"$ source setup_env.sh\"."

# Clean files
rm -rf arch/arm/boot/uImage arch/arm/boot/dts/zynq-motion.dtb

# Build project
[ ! -e .config ] && make ARCH=arm xilinx_zynq_defconfig
make ARCH=arm zynq-motion.dtb
make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage

[ ! -e arch/arm/boot/uImage ] && error "Fail to build project."
cp -af arch/arm/boot/uImage .
cp -af arch/arm/boot/dts/zynq-motion.dtb ./devicetree.dtb

exit 0
