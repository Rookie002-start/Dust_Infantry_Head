# 业务线程装配 — 门禁 TRD_XXX 保持原名（不加 DUST_ 前缀）

if(CONFIG_TRD_GPIO)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gpio/trd_gpio.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/gpio
	)
endif()

if(CONFIG_TRD_CAN)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/can/trd_can_tx.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/can
	)
endif()

if(CONFIG_TRD_TEST)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/trd_test.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/test
	)
endif()

if(CONFIG_TRD_REMOTE)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remote/trd_remote.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/remote
	)
endif()

if(CONFIG_TRD_MCU_INTER)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/mcu_inter/trd_mcu_inter.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/mcu_inter
	)
endif()

if(CONFIG_TRD_INTER_RX)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inter_rx/trd_inter_rx.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/inter_rx
	)
endif()

if(CONFIG_TRD_INTER_BUS)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inter_bus/trd_inter_bus.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/inter_bus
	)
endif()

if(CONFIG_TRD_IMU)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/trd_imu.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/imu
	)
endif()

if(CONFIG_TRD_BOOSTER)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/booster/trd_booster.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/booster
	)
endif()

if(CONFIG_TRD_PC)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pc/trd_pc.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/pc
	)
endif()
