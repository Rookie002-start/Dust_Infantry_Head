# 上下板发送接口（header-only）— 共享给所有业务线程

if(CONFIG_INTER_CMD)
    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()
