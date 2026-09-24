#!/bin/bash
set -e

source ~/zephyrproject/setup_env.sh

# 可选参数：切换调试探针后再烧录（dap-cmsis / stlink / jlink）
#   ./flash.sh           沿用上一次的探针配置
#   ./flash.sh jlink     切到 J-Link(JTAG)
#   ./flash.sh dap-cmsis 切回 CMSIS-DAP（HPM 上会经过
#                        boards/hpm/hpm5361icb/openocd-cmsis-dap-wrapper.sh：
#                        先复位 USB / 解绑 cdc_acm 再调先楫 openocd）
#
# HPM 烧录速度：默认 4000 kHz（见 boards/hpm/hpm5361icb/board.cmake），
# 临时改速度用环境变量，会自动触发一次重新配置：
#   HPM_ADAPTER_SPEED=2000 ./flash.sh dap-cmsis
BUILD_ARGS=()
if [ -n "${1:-}" ]; then
  echo "==> 切换 OpenOCD 探针为 '$1'"
  BUILD_ARGS+=("-DOPENOCD_INTERFACE=$1")
fi
if [ -n "${HPM_ADAPTER_SPEED:-}" ]; then
  echo "==> 使用 HPM_ADAPTER_SPEED=${HPM_ADAPTER_SPEED} kHz"
  BUILD_ARGS+=("-DHPM_ADAPTER_SPEED=${HPM_ADAPTER_SPEED}")
fi
if [ "${#BUILD_ARGS[@]}" -gt 0 ]; then
  west build -d build -- "${BUILD_ARGS[@]}"
fi

west flash
