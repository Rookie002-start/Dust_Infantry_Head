#!/bin/bash
# HPM5361 的 CMSIS-DAP OpenOCD wrapper（Horco CMSIS-DAP, VID:PID=faed:4870 / faed:4873）
#
# 写法参照 boards/st/board_dm_mc02/openocd-cmsis-dap-wrapper.sh，动作同样是四步：
#   1. 杀掉残留的 openocd（它们会占着 /dev/bus/usb，让下一次连接直接失败）
#   2. 复位 USB 设备。这支探针会偶发“假死”：设备/配置/接口描述符都能正常读，
#      但所有字符串描述符读不到，openocd 靠 product string 匹配 "CMSIS-DAP"，于是报
#        Warn : could not read product string for device 0xfaed:0x4870: Operation timed out
#        Error: unable to find a matching CMSIS-DAP device
#      复位（重新枚举）后即可恢复；拔插探针等效。
#   3. 解绑 cdc_acm（探针的虚拟串口被内核占用时更容易进这种假死状态），
#      并且避免它和 openocd 抢这个复合设备。
#   4. 调真正的先楫 OpenOCD（必须带 hpm_xpi flash 驱动）执行 west 传进来的参数，
#      退出后把 cdc_acm 绑回去。
#
# 上面的 sysfs 操作需要 root。若装了 tools/install-hpm-sudoers.sh 提供的免密 helper
# （/usr/local/sbin/hpm-probe-usb），会优先走它，west flash 不再提示密码；
# 没装就退回 `sudo tee` 的老办法（会提示一次密码）。
#
# 真正的 openocd 路径：优先环境变量 HPM_OPENOCD_REAL，否则依次猜
#   ~/openocd-hpm/bin/openocd  ->  ~/hpm-openocd/bin/openocd
# 这两个都是先楫版（带 hpm_xpi）。系统 /usr/bin/openocd 没有 hpm_xpi，
# 拿它烧 HPM 会直接报 "flash driver 'hpm_xpi' not found"。
#
# 用法与 openocd 完全一致（由 boards/hpm/hpm5361icb/bin/openocd 垫片转发）。
# 环境变量：
#   HPM_OPENOCD_REAL=/path/to/openocd   指定真正的 openocd
#   HPM_PROBE_HELPER=/path/to/helper    指定免密 helper（默认 /usr/local/sbin/hpm-probe-usb）
#   HPM_FLASH_NO_RESET=1                跳过 USB 复位（没有 sudo 权限时用，等价于旧的直连行为）
#   HPM_WRAPPER_VERBOSE=1               打印更多过程信息
#
# 非交互场景（无 tty，比如脚本/CI）自动改用 sudo -n，避免卡在密码提示上。

VID="faed"
# 有些 Horco 探针枚举成 4870，有些是 4873，两个都认
PIDS="4870 4873"
USB_DEV=""
NEED_REBIND=""
DO_RESET=1
# 装了免密 helper 就用它（不用输密码），否则退回 sudo tee
PROBE_HELPER="${HPM_PROBE_HELPER:-/usr/local/sbin/hpm-probe-usb}"
[ -x "${PROBE_HELPER}" ] || PROBE_HELPER=""

[ "${HPM_FLASH_NO_RESET:-0}" = "1" ] && DO_RESET=0

# 复位后等探针就绪的最大轮数（每轮 = 一次轻量 openocd self-check + 一次复位）
READY_TRIES="${HPM_READY_TRIES:-3}"

# 非交互时用 sudo -n，防止一直等密码
if [ -t 0 ] && [ -t 1 ]; then
	SUDO="sudo"
else
	SUDO="sudo -n"
fi

warn() { echo "[hpm-wrapper] $*" >&2; }
info() { echo "[hpm-wrapper] $*"; }

# ---- 找出真正的 openocd -----------------------------------------------------
if [ -n "${HPM_OPENOCD_REAL:-}" ]; then
	REAL_OPENOCD="${HPM_OPENOCD_REAL}"
elif [ -x "${HOME}/openocd-hpm/bin/openocd" ]; then
	REAL_OPENOCD="${HOME}/openocd-hpm/bin/openocd"
elif [ -x "${HOME}/hpm-openocd/bin/openocd" ]; then
	REAL_OPENOCD="${HOME}/hpm-openocd/bin/openocd"
else
	REAL_OPENOCD=""
fi
if [ -z "${REAL_OPENOCD}" ] || [ ! -x "${REAL_OPENOCD}" ]; then
	warn "找不到先楫版 openocd（带 hpm_xpi 驱动）"
	warn "请设置 HPM_OPENOCD_REAL=/path/to/openocd 后重试"
	exit 1
fi
[ "${HPM_WRAPPER_VERBOSE:-0}" = "1" ] && info "使用的 openocd: ${REAL_OPENOCD}"

# --version 这类只读调用直接透传，不做 USB 操作（west 在烧录前会问一次版本）
if [ "$#" = "1" ] && { [ "$1" = "--version" ] || [ "$1" = "-v" ]; }; then
	exec "${REAL_OPENOCD}" "$@"
fi

# ---- 工具函数 ---------------------------------------------------------------

# 定位探针的 sysfs 路径（设备号可能在复位后变化，所以每次都重新找）
auto_detect_device() {
	local dev pid _pid
	for dev in /sys/bus/usb/devices/*/; do
		[ -r "${dev}/idVendor" ] || continue
		[ "$(cat "${dev}/idVendor" 2>/dev/null)" = "${VID}" ] || continue
		pid="$(cat "${dev}/idProduct" 2>/dev/null)"
		for _pid in ${PIDS}; do
			if [ "${pid}" = "${_pid}" ]; then
				USB_DEV="$(basename "${dev}")"
				return 0
			fi
		done
	done
	return 1
}

# 杀掉还占着 /dev/bus/usb 的残留 openocd
kill_stale_openocd() {
	local killed=0 pid
	for pid in $(pgrep -x "openocd" 2>/dev/null); do
		if lsof -p "${pid}" 2>/dev/null | grep -q "/dev/bus/usb"; then
			info "发现占用 USB 的残留 openocd (pid=${pid})，先杀掉"
			kill "${pid}" 2>/dev/null || true
			killed=1
		fi
	done
	if [ "${killed}" = "1" ]; then
		sleep 1
	fi
}

# 端口复位 + 重新枚举：探针固件假死时的恢复手段
reset_usb_device() {
	if [ "${DO_RESET}" != "1" ]; then
		info "HPM_FLASH_NO_RESET=1，跳过 USB 复位"
		return 0
	fi

	info "复位 USB 设备 ${USB_DEV} ..."

	if [ -n "${PROBE_HELPER}" ]; then
		if err="$(${SUDO} "${PROBE_HELPER}" reset 2>&1)"; then
			[ -n "${err}" ] && info "${err}"
			auto_detect_device >/dev/null 2>&1 || true
			return 0
		fi
		warn "helper 复位失败：${err}"
		warn "退回 sudo tee 方式"
	fi

	if ! err="$(echo -n "0" | ${SUDO} tee "/sys/bus/usb/devices/${USB_DEV}/authorized" 2>&1 >/dev/null)"; then
		warn "复位 USB 设备失败（${err:-无输出}），跳过复位"
		warn "常见原因是密码没输对 / 没有 sudo 权限 / sysfs 只读"
		warn "若报 'could not read product string'，请手动拔插探针，或执行："
		warn "  echo 0 | sudo tee /sys/bus/usb/devices/${USB_DEV}/authorized"
		warn "  sleep 1; echo 1 | sudo tee /sys/bus/usb/devices/${USB_DEV}/authorized"
		return 1
	fi
	sleep 1
	echo -n "1" | ${SUDO} tee "/sys/bus/usb/devices/${USB_DEV}/authorized" >/dev/null 2>&1 || true

	# 等重新枚举，设备号可能变，重新找一遍
	local i
	for i in $(seq 1 20); do
		if auto_detect_device; then
			info "复位后重新找到探针: ${USB_DEV}"
			return 0
		fi
		sleep 0.5
	done
	warn "复位后没有重新找到探针，继续执行（可能仍会失败）"
	return 0
}

# 解绑 cdc_acm（记下来，退出后绑回去）
release_cdc_acm() {
	local iface_path iface driver drv_name

	# 走 helper 时：先记下当前绑着 cdc_acm 的接口，解绑交给 helper
	if [ -n "${PROBE_HELPER}" ]; then
		local recorded="" out
		for iface_path in "/sys/bus/usb/devices/${USB_DEV}/${USB_DEV}:"*; do
			[ -d "${iface_path}" ] || continue
			iface="$(basename "${iface_path}")"
			driver="$(readlink "${iface_path}/driver" 2>/dev/null || true)"
			[ -n "${driver}" ] || continue
			[ "$(basename "${driver}")" = "cdc_acm" ] || continue
			recorded="${recorded} ${iface}"
		done
		NEED_REBIND="${recorded}"
		if [ -n "${recorded}" ]; then
			if out="$(${SUDO} "${PROBE_HELPER}" unbind-cdc 2>&1)"; then
				info "${out}"
			else
				warn "helper 解绑 cdc_acm 失败：${out}"
			fi
		fi
		return 0
	fi

	for iface_path in "/sys/bus/usb/devices/${USB_DEV}/${USB_DEV}:"*; do
		[ -d "${iface_path}" ] || continue
		iface="$(basename "${iface_path}")"
		driver="$(readlink "${iface_path}/driver" 2>/dev/null || true)"
		[ -n "${driver}" ] || continue
		drv_name="$(basename "${driver}")"
		[ "${drv_name}" = "cdc_acm" ] || continue
		info "解绑 ${iface} (${drv_name})"
		# 只是尽力而为：复位已经能让探针恢复正常时，这一步可有可无，
		# 失败（接口没绑上 / 没权限 / 已经被别的进程解绑）不影响烧录。
		if ! err="$(echo -n "${USB_DEV}:${iface}" | ${SUDO} tee "/sys/bus/usb/drivers/${drv_name}/unbind" 2>&1 >/dev/null)"; then
			warn "解绑 ${iface} 失败（${err:-无输出}），继续执行（不影响烧录）"
		fi
		NEED_REBIND="${NEED_REBIND} ${iface}"
	done
	sleep 0.5
}

rebind_cdc_acm() {
	[ -n "${USB_DEV}" ] || return 0
	[ -n "${NEED_REBIND}" ] || return 0
	local iface

	if [ -n "${PROBE_HELPER}" ]; then
		local out
		if out="$(${SUDO} "${PROBE_HELPER}" bind-cdc 2>&1)"; then
			info "重新绑定: ${out}"
		else
			warn "helper 绑回 cdc_acm 失败：${out}"
		fi
		return 0
	fi

	for iface in ${NEED_REBIND}; do
		if [ ! -L "/sys/bus/usb/devices/${USB_DEV}/${iface}/driver" ]; then
			info "重新绑定 ${iface} -> cdc_acm"
			echo -n "${USB_DEV}:${iface}" | ${SUDO} tee "/sys/bus/usb/drivers/cdc_acm/bind" >/dev/null 2>&1 || true
		fi
	done
}

# 探针是否真的能应答 CMSIS-DAP 命令。
# 注意：设备重新枚举出来（sysfs 节点出现）≠ 探针已就绪 —— 刚复位完立刻跑
# openocd 有时会报 "CMSIS-DAP command CMD_INFO failed"，就是差这一步。
probe_ready() {
	timeout 25 "${REAL_OPENOCD}" \
		-c "adapter driver cmsis-dap" -c "transport select jtag" \
		-c "adapter speed 1000" -c "init" -c "shutdown" 2>&1 |
		grep -q "CMSIS-DAP: Interface ready"
}

# ---- 主流程 -----------------------------------------------------------------

kill_stale_openocd

if ! auto_detect_device; then
	warn "没找到 CMSIS-DAP 探针 (${VID}:${PIDS})，直接调 openocd（由它报错）"
	exec "${REAL_OPENOCD}" "$@"
fi

reset_usb_device
release_cdc_acm

if [ "${DO_RESET}" = "1" ]; then
	attempt=1
	while [ "${attempt}" -le "${READY_TRIES}" ]; do
		if probe_ready; then
			[ "${attempt}" -gt 1 ] && info "探针已就绪（第 ${attempt} 次尝试）"
			break
		fi
		warn "探针还不可用（第 ${attempt}/${READY_TRIES} 次），再复位一次"
		reset_usb_device >/dev/null 2>&1 || true
		sleep 2
		attempt=$((attempt + 1))
	done
	if [ "${attempt}" -gt "${READY_TRIES}" ]; then
		warn "复位 ${READY_TRIES} 次后探针仍不应答 CMSIS-DAP 命令，请拔插探针后再试"
	fi
fi

"${REAL_OPENOCD}" "$@"
RET=$?

rebind_cdc_acm
exit ${RET}
