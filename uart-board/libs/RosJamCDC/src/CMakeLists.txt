cmake_minimum_required(VERSION 3.20)

# Get TinyUSB sources. Note: DCD and HCD drivers are not included
function(rosjam_cdc_sources_get OUTPUT_VAR)
  set(${OUTPUT_VAR}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/usb_descriptors.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/json_serde/serialization.cc
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/json_serde/deserialization.cc
    PARENT_SCOPE
    )
endfunction()

# Add tinyusb to a existing target
function(rosjam_cdc_add TARGET)
  rosjam_cdc_sources_get(ROSJAM_CDC_SRC)
  target_sources(${TARGET} PRIVATE ${ROSJAM_CDC_SRC})
  target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    )
endfunction()
