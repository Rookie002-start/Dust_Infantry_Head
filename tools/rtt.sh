#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# SEGGER RTT 查看服务（主机侧）—— 用 OpenOCD 自带的 RTT 实现，
# 所以 CMSIS-DAP / ST-Link / J-Link 都能用，不必安装 SEGGER 的 J-Link 软件。
#
# 前提：
#   1) 固件打开了 CONFIG_DEBUG_RTT（见 prj.conf），并且已经烧录进板子；
#   2) 调试器（探针）在线，且板子已经跑起来（RTT 控制块由固件在启动时初始化）。
#
# 用法：
#   tools/rtt.sh                    # 自动从 build/CMakeCache.txt 识别板卡，默认 dap-cmsis 探针
#   tools/rtt.sh -i jlink           # 换探针：dap-cmsis / jlink / stlink
#   tools/rtt.sh -v                 # 起完服务后直接接上自带终端查看（Ctrl-C 退出）
#   tools/rtt.sh -p 9098            # 换端口（默认 9090）
#   tools/rtt.sh -e path/zephyr.elf # 指定 ELF（默认 build/zephyr/zephyr.elf）
#   tools/rtt.sh -r                 # 先复位板子再连（默认不动已经在跑的固件）
#   tools/rtt.sh --stop             # 停掉后台的 OpenOCD
#
# 手动查看（推荐另开一个终端，脚本不退出）：
#   nc localhost 9090
#   # 或
#   tools/rtt_view.py

set -euo pipefail

PROJ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${PROJ_DIR}/build"
ELF=""
PORT=9090
IFACE="dap-cmsis"
VIEW=0
STOP=0
DO_RESET=0
PIDFILE="/tmp/infantry_head_rtt_openocd.pid"
LOGFILE="/tmp/infantry_head_rtt_openocd.log"

usage() {
    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"
    exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--build-dir) BUILD_DIR="$2"; shift 2 ;;
        -b|--board)     BOARD="$2";     shift 2 ;;
        -c|--board-cfg) BOARD_CFG="$2"; shift 2 ;;
        -e|--elf)       ELF="$2";       shift 2 ;;
        -i|--interface) IFACE="$2";     shift 2 ;;
        -p|--port)      PORT="$2";      shift 2 ;;
        -v|--view)      VIEW=1;         shift ;;
        -r|--reset)     DO_RESET=1;     shift ;;
        --stop)         STOP=1;         shift ;;
        -h|--help)      usage 0 ;;
        *) echo "未知参数: $1" >&2; usage 1 ;;
    esac
done

if [[ "${STOP}" == "1" ]]; then
    if [[ -f "${PIDFILE}" ]] && kill -0 "$(cat "${PIDFILE}")" 2>/dev/null; then
        kill "$(cat "${PIDFILE}")" && echo "已停止 OpenOCD (pid $(cat "${PIDFILE}"))"
    else
        pkill -f "openocd .*rtt server start ${PORT}" 2>/dev/null && echo "已停止匹配的 OpenOCD" || echo "没有找到运行中的 RTT OpenOCD"
    fi
    rm -f "${PIDFILE}"
    exit 0
fi

# ---- 1. 从 CMakeCache 里推断板卡（可以用 -b/-c 覆盖）----------------------
CACHE="${BUILD_DIR}/CMakeCache.txt"
[[ -f "${CACHE}" ]] || { echo "找不到 ${CACHE}，先 west build 一次，或用 -d 指定 build 目录" >&2; exit 1; }

BOARD="${BOARD:-$(sed -n 's/^BOARD:STRING=//p' "${CACHE}" | head -1)}"
BOARD_CFG="${BOARD_CFG:-$(sed -n 's/^BOARD_CFG:[A-Z]*=//p' "${CACHE}" | head -1)}"
ELF="${ELF:-${BUILD_DIR}/zephyr/zephyr.elf}"

[[ -n "${BOARD}" && -n "${BOARD_CFG}" ]] || { echo "无法从 ${CACHE} 读出 BOARD / BOARD_CFG" >&2; exit 1; }
[[ -f "${ELF}" ]] || { echo "找不到固件 ${ELF}" >&2; exit 1; }

# ---- 2. 找 RTT 控制块地址（ELF 里的 _SEGGER_RTT 符号）---------------------
find_symbol() {
    local sym="$1" tool addr
    for tool in "${ZEPHYR_SDK_INSTALL_DIR:-$HOME/zephyrproject/zephyr-sdk-1.0.1}"/gnu/*/bin/*-readelf; do
        [[ -x "${tool}" ]] || continue
        addr="$("${tool}" -sW "${ELF}" 2>/dev/null | awk -v s="${sym}" '$NF == s { print $2; exit }')"
        if [[ -n "${addr}" ]]; then
            echo "0x${addr}"
            return 0
        fi
    done
    return 1
}

if ! RTT_ADDR="$(find_symbol _SEGGER_RTT)"; then
    echo "在 ${ELF} 里找不到 _SEGGER_RTT 符号 —— 说明固件没开 CONFIG_DEBUG_RTT。" >&2
    echo "请确认 prj.conf 里 CONFIG_DEBUG_RTT=y 并重新 west build。" >&2
    exit 1
fi

# 搜索窗口：控制块本身 + 4 KB 上行缓冲，够 OpenOCD 找到 "SEGGER RTT" 字符串
RTT_SEARCH_SIZE=$((0x1000))

# ---- 3. 组装 OpenOCD 命令 -------------------------------------------------
case "${BOARD_CFG}" in
    hpm*)
        # HPM（RISC-V）：用先楫自带的 openocd（带 hpm_xpi / jlink 支持）
        OPENOCD_BIN="${OPENOCD_BIN:-$HOME/openocd-hpm/bin/openocd}"
        HPM_SDK="${HPM_SDK:-$HOME/Zephyr_HPMicro/sdk_env/hpm_sdk}"
        SDK_GLUE_USER="${SDK_GLUE_USER:-$HOME/Zephyr_HPMicro/sdk_glue_user}"
        OCD_ARGS=(-s "${HPM_SDK}/boards/openocd"
                  -s "${SDK_GLUE_USER}/boards/openocd"
                  -f "${PROJ_DIR}/boards/hpm/${BOARD_CFG}/openocd-debug.cfg")
        ;;
    *)
        # STM32：用工程里的 openocd 配置（与 west flash 完全一致）
        OPENOCD_BIN="${OPENOCD_BIN:-/usr/bin/openocd}"
        case "${IFACE}" in
            jlink) CFG_NAME="openocd-jlink.cfg" ;;
            *)     CFG_NAME="openocd.cfg" ;;
        esac
        CFG="$(ls "${PROJ_DIR}"/boards/*/"${BOARD_CFG}"/"${CFG_NAME}" 2>/dev/null | head -1 || true)"
        if [[ -z "${CFG}" ]]; then
            echo "板卡 ${BOARD_CFG} 没有 ${CFG_NAME}（可用探针：$(ls "${PROJ_DIR}"/boards/*/"${BOARD_CFG}"/openocd*.cfg 2>/dev/null | xargs -n1 basename | tr '\n' ' '))" >&2
            exit 1
        fi
        # dap-cmsis 用 board.cmake 里同一个 wrapper（处理 USB 抢占 / cdc_acm）
        if [[ "${IFACE}" == "dap-cmsis" ]]; then
            WRAPPER="$(ls "${PROJ_DIR}"/boards/*/"${BOARD_CFG}"/openocd-cmsis-dap-wrapper.sh 2>/dev/null | head -1 || true)"
            [[ -n "${WRAPPER}" ]] && OPENOCD_BIN="${WRAPPER}"
        fi
        OCD_ARGS=(-s /usr/share/openocd/scripts -f "${CFG}")
        ;;
esac

[[ -x "${OPENOCD_BIN}" ]] || { echo "找不到可执行的 openocd: ${OPENOCD_BIN}" >&2; exit 1; }

echo "板卡      : ${BOARD} (${BOARD_CFG})"
echo "固件      : ${ELF}"
echo "RTT 控制块: ${RTT_ADDR}"
echo "OpenOCD   : ${OPENOCD_BIN}"
echo "端口      : ${PORT}（另开终端: nc localhost ${PORT}）"
echo "日志      : ${LOGFILE}"
echo

# ---- 4. 起后台 OpenOCD：rtt setup -> rtt start -> rtt server --------------
# 两个坑：
#   1) -c 里的命令一旦返回错误，OpenOCD 会**直接退出**（退出码 1）；
#      而"目标已经在运行"时 resume 恰好会报 "target not halted" 错误，
#      所以 resume 必须用 catch 包起来。
#   2) 目标可能处于 halt 状态（比如 VS Code 调试停下过），这时候 CPU 不跑，
#      RTT 就没有新数据；所以最后再补一发"如果还停着就 resume"。
# 默认不复位板子（避免打断正在跑的实验），需要复位时加 -r。
OCD_CMDS=(-c "init")
if [[ "${DO_RESET}" == "1" ]]; then
    # 复位并让固件开始跑，否则下面的 rtt setup 找不到还没初始化的控制块
    OCD_CMDS+=(-c "reset halt" -c 'if {[catch {resume} msg]} { echo "resume: $msg" }')
fi
OCD_CMDS+=(-c "rtt setup ${RTT_ADDR} ${RTT_SEARCH_SIZE} \"SEGGER RTT\"")
# 控制块由固件启动时初始化，复位后可能要等几毫秒：失败就 100ms 重试，最多 3s
OCD_CMDS+=(-c 'set rtt_ok 0
for {set i 0} {$i < 30} {incr i} {
    if {[catch {rtt start} msg] == 0} { set rtt_ok 1; break }
    sleep 100
}
if {!$rtt_ok} { echo "rtt start failed: $msg" }')
OCD_CMDS+=(-c "rtt server start ${PORT} 0"
           -c 'if {[catch {resume} msg]} { echo "resume: $msg" }')

"${OPENOCD_BIN}" "${OCD_ARGS[@]}" "${OCD_CMDS[@]}" >"${LOGFILE}" 2>&1 &
OCD_PID=$!
echo "${OCD_PID}" > "${PIDFILE}"

cleanup() {
    if kill -0 "${OCD_PID}" 2>/dev/null; then
        kill "${OCD_PID}" 2>/dev/null || true
        wait "${OCD_PID}" 2>/dev/null || true
    fi
    rm -f "${PIDFILE}"
}
trap cleanup EXIT INT TERM

# 等端口起来（最多 ~8s）
for _ in $(seq 1 40); do
    if (exec 3<>"/dev/tcp/127.0.0.1/${PORT}") 2>/dev/null; then
        exec 3>&- 3<&-
        break
    fi
    if ! kill -0 "${OCD_PID}" 2>/dev/null; then
        echo "OpenOCD 退出了，日志：${LOGFILE}" >&2
        grep -iE "error|not found|failed|no control block" "${LOGFILE}" | tail -20 >&2 || tail -20 "${LOGFILE}" >&2
        exit 1
    fi
    sleep 0.2
done

if grep -qiE "no control block found|not configured" "${LOGFILE}"; then
    echo "OpenOCD 没找到 RTT 控制块 —— 确认固件已烧录并且正在运行。" >&2
    grep -iE "rtt" "${LOGFILE}" | tail -10 >&2
    exit 1
fi

echo "RTT 服务已就绪，Ctrl-C 结束。"
echo "  查看方式 1：另开终端执行  nc localhost ${PORT}"
echo "  查看方式 2：另开终端执行  tools/rtt_view.py -p ${PORT}"
echo "  查看方式 3：VS Code 里用带 RTT 的 cortex-debug 配置（见 docs/segger_rtt.md）"

if [[ "${VIEW}" == "1" ]]; then
    python3 "${PROJ_DIR}/tools/rtt_view.py" -p "${PORT}" || true
else
    wait "${OCD_PID}"
fi
