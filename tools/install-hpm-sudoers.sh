#!/bin/bash
# 安装 HPM5361 CMSIS-DAP 烧录用到的免密 sudo 规则。
#
#   sudo ./tools/install-hpm-sudoers.sh            # 给当前 sudo 调用者安装
#   sudo ./tools/install-hpm-sudoers.sh rookie     # 指定用户名
#
# 做两件事：
#   1) 把 tools/hpm-probe-usb.sh 装到 /usr/local/sbin/hpm-probe-usb (root:root 0755)
#   2) 生成 /etc/sudoers.d/hpm-cmsis-dap (root:root 0440)，只放开这一个 helper 的
#      reset / unbind-cdc / bind-cdc 三个子命令 —— 不是放开 sudo tee，也不是放开
#      整个 sudo；删掉那个文件即可完全撤销。
#
# 装完后 boards/hpm/hpm5361icb/openocd-cmsis-dap-wrapper.sh 会自动优先用这个
# helper，west flash 不再提示密码。
set -euo pipefail

HELPER_SRC="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)/hpm-probe-usb.sh"
HELPER_DST="/usr/local/sbin/hpm-probe-usb"
SUDOERS_FILE="/etc/sudoers.d/hpm-cmsis-dap"

TARGET_USER="${1:-${SUDO_USER:-$(id -un)}}"

if [ "$(id -u)" != "0" ]; then
	echo "请用 root 运行： sudo $0 [用户名]" >&2
	exit 1
fi

if [ ! -r "${HELPER_SRC}" ]; then
	echo "找不到 ${HELPER_SRC}" >&2
	exit 1
fi

if ! id "${TARGET_USER}" >/dev/null 2>&1; then
	echo "用户不存在：${TARGET_USER}" >&2
	exit 1
fi

command -v visudo >/dev/null 2>&1 || { echo "找不到 visudo（sudo 包没装？）" >&2; exit 1; }

echo "==> 安装 helper: ${HELPER_DST}"
install -o root -g root -m 0755 "${HELPER_SRC}" "${HELPER_DST}"

echo "==> 生成 sudoers 规则: ${SUDOERS_FILE}（用户 ${TARGET_USER}）"
TMP_FILE="$(mktemp)"
trap 'rm -f "${TMP_FILE}"' EXIT
cat > "${TMP_FILE}" <<EOF
# HPM5361 CMSIS-DAP 烧录：允许 ${TARGET_USER} 免密复位 Horco 探针、绑/解绑它的 cdc_acm。
# 由 tools/install-hpm-sudoers.sh 生成；删除本文件即可撤销。
${TARGET_USER} ALL=(root) NOPASSWD: ${HELPER_DST} reset, ${HELPER_DST} unbind-cdc, ${HELPER_DST} bind-cdc
EOF

# 先用 visudo 校验语法，通过了再落到 /etc/sudoers.d
if ! visudo -cf "${TMP_FILE}"; then
	echo "sudoers 语法校验失败，未做任何改动" >&2
	exit 1
fi

install -o root -g root -m 0440 "${TMP_FILE}" "${SUDOERS_FILE}"

echo
echo "安装完成。自检（只查 sudoers 是否放行，不会真的复位探针）："
if sudo -n -l "${HELPER_DST}" reset >/dev/null 2>&1; then
	echo "  ✔ ${TARGET_USER} 免密调用已生效：sudo ${HELPER_DST} reset / unbind-cdc / bind-cdc"
else
	echo "  ✘ 免密调用没生效，请检查 ${SUDOERS_FILE}（可能被别的 sudoers 规则覆盖）" >&2
	exit 1
fi

echo
echo "撤销方式： sudo rm -f ${SUDOERS_FILE} ${HELPER_DST}"
