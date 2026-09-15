# SPDX-License-Identifier: Apache-2.0

# 板卡专属全局 include（根 CMakeLists 的 foreach 统一 zephyr_include_directories）
# CMSIS Core 头文件由 Zephyr 模块系统自动提供（CONFIG_HAS_CMSIS_CORE=y）
set(BOARD_GLOBAL_INCLUDES
  "${ZEPHYR_USER_DIR}/platform/cmsis"
)

# CMSIS v5.7+ 将 SHPR 重命名为 SHP，Zephyr 源码仍用旧名
add_compile_definitions(SHPR=SHP)

set(BOARD_FLASH_RUNNER openocd)

# 调试探针选择，覆盖方式：cmake -DOPENOCD_INTERFACE=<probe>
#   dap-cmsis : CMSIS-DAP + USB wrapper（Horco faed:4873 workaround）【默认】
#   stlink    : ST-Link（SWD）
#   jlink     : SEGGER J-Link（JTAG）
if(NOT OPENOCD_INTERFACE)
  set(OPENOCD_INTERFACE "dap-cmsis")
endif()

set(BOARD_CFG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/boards")

# 每个探针对应一份 OpenOCD 配置；只有 CMSIS-DAP 需要 wrapper 绕过 USB 抢占
if(OPENOCD_INTERFACE STREQUAL "dap-cmsis")
  set(_ocd_cfg_name "openocd.cfg")
  file(GLOB ocd_wrapper "${BOARD_CFG_DIR}/*/${BOARD_CFG}/openocd-cmsis-dap-wrapper.sh")
  if(ocd_wrapper)
    set(OPENOCD "${ocd_wrapper}"
        CACHE FILEPATH "OpenOCD wrapper for CMSIS-DAP" FORCE)
  endif()
else()
  if(OPENOCD_INTERFACE STREQUAL "jlink")
    set(_ocd_cfg_name "openocd-jlink.cfg")
  else()
    set(_ocd_cfg_name "openocd.cfg")
  endif()
  # FORCE：从 dap-cmsis 切走时必须把 wrapper 路径覆盖掉
  set(OPENOCD "/usr/bin/openocd" CACHE FILEPATH "" FORCE)
endif()

macro(app_set_runner_args)
  get_property(_ocd_done GLOBAL PROPERTY _OCD_STM32_DONE)
  if(NOT _ocd_done)
    set_property(GLOBAL PROPERTY _OCD_STM32_DONE TRUE)
    board_runner_args(openocd "--openocd-search=/usr/share/openocd/scripts" "--cmd-reset-halt=reset halt")
    file(GLOB ocd_cfg "${BOARD_CFG_DIR}/*/${BOARD_CFG}/${_ocd_cfg_name}")
    if(ocd_cfg)
      board_runner_args(openocd "--config=${ocd_cfg}")
    endif()
  endif()
endmacro()
