cmake_minimum_required(VERSION 3.20)
enable_language(C ASM)

find_package(Git QUIET)
if(GIT_FOUND)
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/libs/tinyusb/src/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/libs/mpack_git/tools/amalgamate.sh")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

set(ROSJAM_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/rosjam.c
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_descriptors.c
  ${CMAKE_CURRENT_SOURCE_DIR}/buffers.c
  ${CMAKE_CURRENT_SOURCE_DIR}/json_serde/serialization.c
  ${CMAKE_CURRENT_SOURCE_DIR}/json_serde/deserialization.c

  ${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/portable/st/typec/typec_stm32.c
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src "${CMAKE_CURRENT_BINARY_DIR}/tinyusb_build")
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
    cobs-c
    mpack
    # Add user defined libraries
)
