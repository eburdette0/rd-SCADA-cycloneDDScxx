set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv71)


set(toolchainpath c:/Users/eburdette/AppData/Local/msys64/home/eburdette/uei/ueipac-arm64-zynq_5.1.1.136/sysroots)
set(CMAKE_C_COMPILER ${toolchainpath}/x86_64-petalinux-mingw32/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-gcc)
set(CMAKE_CXX_COMPILER ${toolchainpath}/x86_64-petalinux-mingw32/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-g++)

set(CMAKE_SYSROOT ${toolchainpath}/aarch64-xilinx-linux)


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)


