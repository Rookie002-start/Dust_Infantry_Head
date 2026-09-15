# SPDX-License-Identifier: Apache-2.0

# 板卡专属全局 include（根 CMakeLists 的 foreach 统一 zephyr_include_directories）
# CMSIS Core 头文件由 Zephyr 模块系统自动提供（CONFIG_HAS_CMSIS_CORE=y）
set(BOARD_GLOBAL_INCLUDES
  "${ZEPHYR_USER_DIR}/platform/cmsis"
)

# CMSIS v5.7+ 将 SHPR 重命名为 SHP，Zephyr 源码仍用旧名          
add_compile_definitions(SHPR=SHP) 

set(BOARD_FLASH_RUNNER openocd)
set(OPENOCD "/usr/bin/openocd" CACHE FILEPATH "")

set(BOARD_CFG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/boards")

macro(app_set_runner_args)
  get_property(_ocd_done GLOBAL PROPERTY _OCD_STM32_DONE)
  if(NOT _ocd_done)
    set_property(GLOBAL PROPERTY _OCD_STM32_DONE TRUE)
    board_runner_args(openocd "--openocd-search=/usr/share/openocd/scripts")
    file(GLOB ocd_cfg "${BOARD_CFG_DIR}/*/${BOARD_CFG}/openocd.cfg")
    if(ocd_cfg)
      board_runner_args(openocd "--config=${ocd_cfg}")
    endif()
  endif()
endmacro()
