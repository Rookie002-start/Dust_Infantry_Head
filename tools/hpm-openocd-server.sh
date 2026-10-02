#!/bin/bash
# 起一个 HPM5361 的 OpenOCD gdb 服务器（localhost:3333），供 VS Code 的
# "HPM5361 附加 (DAP JTAG)" 使用：attach 类型不会自己拉服务器，要先跑这个。
#
#   ./tools/hpm-openocd-server.sh        # 前台运行，Ctrl-C 结束
#
# 内部走 boards/hpm/hpm5361icb/openocd-cmsis-dap-wrapper.sh：
# 杀残留 openocd -> 复位探针 USB -> 解绑 cdc_acm -> 确认探针能应答 -> 起 openocd。
# Ctrl-C 时会一并收掉 openocd 并把 cdc_acm 绑回去。
set -u

PROJ_DIR="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
SDK=/home/rookie/Zephyr_HPMicro/sdk_env/hpm_sdk/boards/openocd
SDK_USER=/home/rookie/Zephyr_HPMicro/sdk_glue_user/boards/openocd

# 注意：这句里不要出现 openocd 自己的 "Listening on port ... for gdb connections"
# 字样，否则用 grep 判断服务器是否就绪会被这行本身误匹配。
echo "HPM5361 OpenOCD gdb 服务器启动中，等 openocd 打印 gdb 监听端口后即可在 VS Code 里 attach ..."
exec "${PROJ_DIR}/boards/hpm/hpm5361icb/openocd-cmsis-dap-wrapper.sh" \
	-s "${SDK}" \
	-s "${SDK_USER}" \
	-f "${PROJ_DIR}/boards/hpm/hpm5361icb/openocd-debug.cfg"
