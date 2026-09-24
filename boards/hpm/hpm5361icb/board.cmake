# HPM 平台：把 sdk_glue / sdk_glue_user 追加进 BOARD/SOC/DTS 根路径，并挂为 Zephyr 模块

# ---- 使用先楫专用 OpenOCD（带 hpm_xpi flash 驱动）----------------------------
# 系统自带的 openocd（apt / Zephyr SDK）没有 hpm_xpi 驱动，west flash 会直接报
#   Error: flash driver 'hpm_xpi' not found
# sdk_glue_user 的 board.cmake 是用 find_program() 从 PATH 里找 openocd 的，
# 所以这里在 find_package(Zephyr) 之前把它插到 PATH 最前面。
#   优先 ~/openocd-hpm      —— 自编译版，hpm_xpi + jlink 都有（用 J-Link 烧录用这个）
#   其次 ~/hpm-openocd      —— 先楫官方版，有 cmsis-dap/ftdi 但没有 jlink
#   可用 HPM_OPENOCD_DIR 覆盖
if(DEFINED ENV{HPM_OPENOCD_DIR})
  set(_HPM_OPENOCD_BIN "$ENV{HPM_OPENOCD_DIR}/bin")
elseif(EXISTS "$ENV{HOME}/openocd-hpm/bin/openocd")
  set(_HPM_OPENOCD_BIN "$ENV{HOME}/openocd-hpm/bin")
else()
  set(_HPM_OPENOCD_BIN "$ENV{HOME}/hpm-openocd/bin")
endif()
if(EXISTS "${_HPM_OPENOCD_BIN}/openocd")
  set(ENV{PATH} "${_HPM_OPENOCD_BIN}:$ENV{PATH}")
  message(STATUS "使用先楫 OpenOCD: ${_HPM_OPENOCD_BIN}/openocd")
else()
  message(WARNING "未找到先楫 OpenOCD (${_HPM_OPENOCD_BIN}/openocd)，烧录会报 hpm_xpi 驱动缺失")
endif()

# ---- CMSIS-DAP 探针走 wrapper（思路同 ST 板 board_dm_mc02）-------------------
# sdk_glue_user 的 board.cmake 会 unset(OPENOCD CACHE) 再用 find_program() 从
# PATH 里重新找 openocd，所以在工程里直接 set(OPENOCD ...) 会被覆盖；
# 这里把带同名垫片的目录插到 PATH 最前面，find_program() 找到的就是 wrapper。
# wrapper 负责：杀残留 openocd + 复位 USB（探针字符串描述符假死）+ 解绑 cdc_acm，
# 再调真正的 ~/openocd-hpm/bin/openocd。
# 只有 CMSIS-DAP（dap-cmsis / cmsis_dap，也是默认值）才走 wrapper：
#   jlink / ft2232 等直连，避免每次烧录都去动 USB。
set(_HPM_WRAPPER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/boards/hpm/hpm5361icb/bin")
if(NOT DEFINED OPENOCD_INTERFACE OR
   OPENOCD_INTERFACE STREQUAL "dap-cmsis" OR
   OPENOCD_INTERFACE STREQUAL "cmsis_dap")
  if(EXISTS "${_HPM_WRAPPER_DIR}/openocd")
    set(ENV{PATH} "${_HPM_WRAPPER_DIR}:$ENV{PATH}")
    message(STATUS "CMSIS-DAP 探针走 wrapper: ${_HPM_WRAPPER_DIR}/openocd")
  else()
    message(WARNING "未找到 CMSIS-DAP wrapper (${_HPM_WRAPPER_DIR}/openocd)")
  endif()
endif()

if(DEFINED ENV{SDK_GLUE_DIR})
  set(SDK_GLUE_DIR "$ENV{SDK_GLUE_DIR}")
else()
  set(SDK_GLUE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Zephyr_HPMicro/sdk_glue")
endif()
set(SDK_GLUE_USER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Zephyr_HPMicro/sdk_glue_user")
if(EXISTS "${SDK_GLUE_DIR}")
  list(APPEND BOARD_ROOT "${SDK_GLUE_DIR}")
  list(APPEND SOC_ROOT   "${SDK_GLUE_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_DIR}/dts")
  list(APPEND BOARD_ROOT "${SDK_GLUE_USER_DIR}")
  list(APPEND SOC_ROOT   "${SDK_GLUE_USER_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_USER_DIR}")
  list(APPEND DTS_ROOT   "${SDK_GLUE_USER_DIR}/dts")
  list(APPEND ZEPHYR_EXTRA_MODULES "${SDK_GLUE_DIR}")
endif()

macro(app_set_runner_args)
  # 烧录/调试的 JTAG 时钟（kHz）。默认 4000，实测依据（Horco CMSIS-DAP + 本机 VMware）：
  #   读 16KB 吞吐 : 500k=2.22 / 1000k=2.49 / 2000k=3.20 / 4000k=3.58 / 8000k=3.59 KiB/s
  #   整片 120KB 写入: 500k = 60.07s (1.947 KiB/s) -> 4000k = 40.56s (2.884 KiB/s)
  #   4000 以上已饱和：瓶颈是探针每次访问约 1ms 的 USB 往返（全速 USB + UHCI 穿透），
  #   不是 JTAG 时钟（换 riscv set_mem_access 的 progbuf/abstract 更慢，分别是
  #   3.21 / 0.61 KiB/s）。想要质变只能换探针（J-Link：./flash.sh jlink）。
  # 临时改：HPM_ADAPTER_SPEED=2000 ./flash.sh dap-cmsis
  #   或    west build -d build -- -DHPM_ADAPTER_SPEED=2000 && west flash
  if(NOT DEFINED HPM_ADAPTER_SPEED)
    if(DEFINED ENV{HPM_ADAPTER_SPEED})
      set(HPM_ADAPTER_SPEED "$ENV{HPM_ADAPTER_SPEED}")
    else()
      set(HPM_ADAPTER_SPEED 4000)
    endif()
  endif()
  if(HPM_ADAPTER_SPEED GREATER 4000)
    message(STATUS "HPM_ADAPTER_SPEED=${HPM_ADAPTER_SPEED} kHz 已超过本探针的饱和点(4000)，不会更快")
  endif()
  board_runner_args(openocd "--cmd-pre-init=adapter speed ${HPM_ADAPTER_SPEED}")
endmacro()
