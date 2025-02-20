#!/bin/bash

export SIFLI_SDK="$(pwd)/"
export PYTHONPATH="$PYTHONPATH:$SIFLI_SDK/tools/build:$SIFLI_SDK/tools/build/default"
export PATH="$PATH:$SIFLI_SDK/tools/scripts;$SIFLI_SDK/tools/kconfig"

# 默认使用 Keil 编译工具
export RTT_CC="keil"
export RTT_EXEC_PATH="/c/Keil_v5"

# 如果传入参数为 gcc，则切换到 GCC 环境
if [ "$1" = "gcc" ]; then
    export RTT_CC="gcc"
    # 根据实际 GCC 工具链所在位置修改 RTT_EXEC_PATH
    export RTT_EXEC_PATH="/Users/halfsweet/arm-none-eabi-gcc-14.2.1-1.1/bin"
    export PATH="$PATH:$RTT_EXEC_PATH"
fi