cmake_minimum_required(VERSION 3.20)
enable_language(C ASM)
# Submodule update from https://cliutils.gitlab.io/modern-cmake/chapters/projects/submodule.html
find_package(Git QUIET)
if(GIT_FOUND)
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Updating submodules")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../libs/tinyusb/src/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_git/tools/amalgamate.sh")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack/src/mpack/mpack.c" OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack/src/mpack/mpack.h")
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_git/tools/amalgamate.sh" AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
        execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_git/tools/amalgamate.sh" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_git")
        file(REMOVE_RECURSE "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack")
        file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_git/.build/amalgamation" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../libs/")
        file(RENAME "${CMAKE_CURRENT_SOURCE_DIR}/../libs/amalgamation" "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack")
    
    else()
        file(REMOVE_RECURSE "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack")
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_temp")
        file(DOWNLOAD https://github.com/ludocode/mpack/releases/download/v1.1.1/mpack-amalgamation-1.1.1.tar.gz "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_temp/mpack.tar.gz") # change url when updating lib
        
        file(ARCHIVE_EXTRACT INPUT "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_temp/mpack.tar.gz" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_temp/")
        file(GLOB MPACK_AMALGAMATION_DIR_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_temp/mpack-amalgamation-*")
        message(STATUS "Unpacked mpack amalgamation package to ${MPACK_AMALGAMATION_DIR_PATH}")
        message(STATUS "Copying amalgamation to correct directory ${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack")
        get_filename_component(MPACK_AMALGAMATION_DIR_NAME "${MPACK_AMALGAMATION_DIR_PATH}" NAME)
        message(STATUS "Found version ${MPACK_AMALGAMATION_DIR_NAME}")
        file(COPY ${MPACK_AMALGAMATION_DIR_PATH} DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../libs/")
        file(RENAME "${CMAKE_CURRENT_SOURCE_DIR}/../libs/${MPACK_AMALGAMATION_DIR_NAME}" "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack")
        message(STATUS "Cleaning temp directory")
        file(REMOVE_RECURSE "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack_temp")
        message(STATUS "Copying CMakeLists.txt for mpack")
        
    endif()
else()
    message(STATUS "mpack already available, skipping amalgamation package generation/download")
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack/CMakeLists.txt")
    file(COPY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/mpack.cmake" "${CMAKE_CURRENT_SOURCE_DIR}/../libs/mpack/CMakeLists.txt")
else()
    message(STATUS "mpack CMakeLists.txt has already been copied")
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
