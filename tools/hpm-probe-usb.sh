#!/bin/bash
# Horco CMSIS-DAP（VID:PID = faed:4870 / faed:4873）专用 USB 操作 helper。
#
# 由 tools/install-hpm-sudoers.sh 安装到 /usr/local/sbin/hpm-probe-usb，
# 配合 /etc/sudoers.d/hpm-cmsis-dap 让 board.cmake 里的 CMSIS-DAP wrapper
# 能免密复位探针（否则每次 west flash 都要输一次 sudo 密码）。
#
# 刻意不接受任何路径参数：只按 VID/PID 自己找设备，sysfs 路径都写死在脚本内部，
# 这样 NOPASSWD 规则不会变成"任意文件写入 root 权限"的口子。
#
# 用法（reset/unbind-cdc/bind-cdc 需要 root；status 不需要）：
#   hpm-probe-usb status        # 打印探针设备号与各接口的驱动绑定情况
#   hpm-probe-usb reset         # authorized 0 -> 1，强制重新枚举（治字符串描述符假死）
#   hpm-probe-usb unbind-cdc    # 解绑探针的 cdc_acm 虚拟串口
#   hpm-probe-usb bind-cdc      # 把探针的 CDC 接口绑回 cdc_acm

set -u

VID="faed"
PIDS="4870 4873"

usage() {
	echo "usage: $0 {status|reset|unbind-cdc|bind-cdc}" >&2
	exit 2
}

die() { echo "hpm-probe-usb: $*" >&2; exit 1; }

need_root() {
	[ "$(id -u)" = "0" ] || die "需要 root 权限（用 sudo 运行）"
}

# 找出探针的 sysfs 设备名（如 1-2.1），找不到返回 1
find_probe() {
	local dev pid _pid
	for dev in /sys/bus/usb/devices/*/; do
		[ -r "${dev}/idVendor" ] || continue
		[ "$(cat "${dev}/idVendor" 2>/dev/null)" = "${VID}" ] || continue
		pid="$(cat "${dev}/idProduct" 2>/dev/null)"
		for _pid in ${PIDS}; do
			if [ "${pid}" = "${_pid}" ]; then
				basename "${dev}"
				return 0
			fi
		done
	done
	return 1
}

# 列出探针的 CDC 接口（class 02 = CDC 通信，0a = CDC 数据）
list_cdc_ifaces() {
	local dev="$1" iface cls
	for iface in "/sys/bus/usb/devices/${dev}:"*; do
		[ -d "${iface}" ] || continue
		cls="$(cat "${iface}/bInterfaceClass" 2>/dev/null)"
		case "${cls}" in
		02 | 0a) basename "${iface}" ;;
		esac
	done
}

iface_driver() {
	local link
	link="$(readlink "/sys/bus/usb/devices/$1/driver" 2>/dev/null || true)"
	[ -n "${link}" ] && basename "${link}"
}

[ "$#" = "1" ] || usage

DEV="$(find_probe || true)"

case "$1" in
status)
	if [ -z "${DEV}" ]; then
		echo "未找到探针 (${VID}:${PIDS// /, })"
		exit 1
	fi
	echo "探针: ${DEV}"
	for iface in "/sys/bus/usb/devices/${DEV}:"*; do
		[ -d "${iface}" ] || continue
		name="$(basename "${iface}")"
		cls="$(cat "${iface}/bInterfaceClass" 2>/dev/null)"
		drv="$(iface_driver "${name}")"
		printf '  %-12s class=%-4s driver=%s\n' "${name}" "${cls}" "${drv:--}"
	done
	;;

reset)
	need_root
	[ -n "${DEV}" ] || die "未找到探针 (${VID})"
	echo 0 > "/sys/bus/usb/devices/${DEV}/authorized" || die "写 authorized=0 失败"
	sleep 1
	echo 1 > "/sys/bus/usb/devices/${DEV}/authorized" || die "写 authorized=1 失败"

	# 等重新枚举（设备号可能变化，重新找）
	found=""
	for i in $(seq 1 20); do
		found="$(find_probe || true)"
		[ -n "${found}" ] && break
		sleep 0.5
	done
	[ -n "${found}" ] || die "复位后没有重新枚举出探针，可能需要物理拔插"
	echo "reset ok: ${found}"
	;;

unbind-cdc)
	need_root
	[ -n "${DEV}" ] || die "未找到探针 (${VID})"
	rc=0
	for iface in $(list_cdc_ifaces "${DEV}"); do
		[ "$(iface_driver "${iface}")" = "cdc_acm" ] || continue
		if echo -n "${iface}" > /sys/bus/usb/drivers/cdc_acm/unbind 2>/dev/null; then
			echo "unbound ${iface}"
		else
			echo "unbind ${iface} failed" >&2
			rc=1
		fi
	done
	exit ${rc}
	;;

bind-cdc)
	need_root
	[ -n "${DEV}" ] || die "未找到探针 (${VID})"
	rc=0
	for iface in $(list_cdc_ifaces "${DEV}"); do
		[ -z "$(iface_driver "${iface}")" ] || continue
		if echo -n "${iface}" > /sys/bus/usb/drivers/cdc_acm/bind 2>/dev/null; then
			echo "bound ${iface}"
		else
			echo "bind ${iface} failed" >&2
			rc=1
		fi
	done
	exit ${rc}
	;;

*)
	usage
	;;
esac
