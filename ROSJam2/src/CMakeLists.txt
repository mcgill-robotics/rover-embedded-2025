cmake_minimum_required(VERSION 3.20)
enable_language(C CXX ASM)

set(ROSJAM_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/rosjam.c
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_descriptors.c
  ${CMAKE_CURRENT_SOURCE_DIR}/buffers.c
  ${CMAKE_CURRENT_SOURCE_DIR}/json_serde/serialization.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/json_serde/deserialization.cc

  ${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/portable/st/typec/typec_stm32.c
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src "${CMAKE_CURRENT_BINARY_DIR}/tinyusb_build")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/ArduinoJson "${CMAKE_CURRENT_BINARY_DIR}/ArduinoJson_build")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack "${CMAKE_CURRENT_BINARY_DIR}/mpack_build")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../cobs/cobs-c/src "${CMAKE_CURRENT_BINARY_DIR}/cobs_build")
add_library(rosjam2 STATIC 
  ${ROSJAM_SRC}
)
tinyusb_target_add(rosjam2)
target_sources(rosjam2 PUBLIC ${ROSJAM_SRC})
target_include_directories(rosjam2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# add_dependencies(rosjam2
#     ${STM32_CUBE_MX}
#     ArduinoJson
# )
target_link_libraries(rosjam2
    stm32cubemx
    ArduinoJson
    cobs-c
    mpack
    # Add user defined libraries
)
