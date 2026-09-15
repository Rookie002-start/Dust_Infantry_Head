if(CONFIG_DUST_TPC_TO_MCU_TX)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/to_mcu)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/to_mcu/to_mcu_tx.cpp)
endif()

if(CONFIG_DUST_TPC_FROM_BODY)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/from_body)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/from_body/from_body.cpp)
endif()
