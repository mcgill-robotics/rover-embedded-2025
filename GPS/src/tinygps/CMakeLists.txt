cmake_minimum_required(VERSION 3.22)

add_library(tinygps STATIC
    ${CMAKE_CURRENT_LIST_DIR}/tinygps.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tinyubx.c
)

target_include_directories(tinygps PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(tinygps PUBLIC stm32cubemx)
