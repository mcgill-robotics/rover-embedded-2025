cmake_minimum_required(VERSION 3.20)
enable_language(C ASM)

set(MPACK_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mpack/mpack.c
)

add_library(mpack STATIC 
  ${MPACK_SRC}
)

target_sources(mpack PUBLIC ${MPACK_SRC})

target_include_directories(mpack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/mpack)
