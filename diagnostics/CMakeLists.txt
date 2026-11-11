if(CONFIG_LATENCY_TRACE)
    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_sources(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/latency_trace.cpp
    )
endif()
