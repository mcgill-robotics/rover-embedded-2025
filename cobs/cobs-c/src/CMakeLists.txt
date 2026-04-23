cmake_minimum_required(VERSION 3.20)
enable_language(C CXX ASM)

set(COBS_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/cobs.c
)

add_library(cobs-c STATIC 
  ${COBS_SRC}
)

target_sources(cobs-c PUBLIC ${COBS_SRC})
target_include_directories(cobs-c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
