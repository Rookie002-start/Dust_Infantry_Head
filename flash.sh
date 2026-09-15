#!/bin/bash
set -e

source ~/zephyrproject/setup_env.sh

# 可选参数：切换调试探针后再烧录（dap-cmsis / stlink / jlink）
#   ./flash.sh           沿用上一次的探针配置
#   ./flash.sh jlink     切到 J-Link(JTAG)
#   ./flash.sh dap-cmsis 切回 CMSIS-DAP
if [ -n "$1" ]; then
  echo "==> 切换 OpenOCD 探针为 '$1'"
  west build -d build -- -DOPENOCD_INTERFACE="$1"
fi

west flash
