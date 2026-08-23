cmake_minimum_required(VERSION 3.20)
enable_language(C CXX ASM)

file(GLOB_RECURSE TOF_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
# set(ROSJAM_SRC
#   ${CMAKE_CURRENT_SOURCE_DIR}/rosjam.c
#   ${CMAKE_CURRENT_SOURCE_DIR}/usb_descriptors.c

#   ${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
#   ${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/portable/st/stm32_fsdev/fsdev_common.c
#   ${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/portable/st/typec/typec_stm32.c
# )

add_library(vl53l3cx STATIC 
  ${TOF_SRC}
)
target_sources(vl53l3cx PUBLIC ${TOF_SRC})
target_include_directories(vl53l3cx PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/porting
    ${CMAKE_CURRENT_SOURCE_DIR}
)
# add_dependencies(rosjam2
#     ${STM32_CUBE_MX}
#     ArduinoJson
# )
# target_link_libraries(rosjam2
#     stm32cubemx
#     # Add user defined libraries
# )
