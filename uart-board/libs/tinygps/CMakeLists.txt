cmake_minimum_required(VERSION 3.20)

function(tiny_gps_sources_get OUTPUT_VAR)
  set(${OUTPUT_VAR}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/tinygps.cpp
    PARENT_SCOPE
    )
endfunction()

# Add tinyusb to a existing target
function(tiny_gps_add TARGET)
  tiny_gps_sources_get(TINY_GPS_SRC)
  target_sources(${TARGET} PRIVATE ${TINY_GPS_SRC})
  target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    )
endfunction()
