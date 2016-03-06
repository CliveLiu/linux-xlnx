#!/bin/bash
#
# Copyright (c) 2016 CliveLiu.
# All rights reserved.
#

TOOLCHAIN_DIR=/opt/Xilinx

export ARCH=arm
export CROSS_COMPILE=arm-xilinx-linux-gnueabi-

source $TOOLCHAIN_DIR/SDK/2015.4/.settings64-XSCT_Core_Tools.sh
source $TOOLCHAIN_DIR/DocNav/.settings64-DocNav.sh
