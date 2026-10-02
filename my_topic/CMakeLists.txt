if(CONFIG_DUST_TPC_FROM_BODY)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/from_body)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/from_body/from_body.cpp)
endif()

if(CONFIG_DUST_TPC_PC_TO)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pc_to)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pc_to/pc_to.cpp)
endif()
