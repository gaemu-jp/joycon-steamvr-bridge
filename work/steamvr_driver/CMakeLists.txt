cmake_minimum_required(VERSION 3.20)
project(joycon_steamvr_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point OPENVR_SDK_ROOT at a local checkout of ValveSoftware/openvr.
if(NOT DEFINED OPENVR_SDK_ROOT)
  message(FATAL_ERROR "Set OPENVR_SDK_ROOT to the OpenVR SDK path")
endif()

add_library(driver_joycon SHARED
  joycon_driver.cpp
)

target_include_directories(driver_joycon PRIVATE
  "${OPENVR_SDK_ROOT}/headers"
)

target_compile_definitions(driver_joycon PRIVATE
  WIN32_LEAN_AND_MEAN
  NOMINMAX
)

if (WIN32)
  target_link_libraries(driver_joycon PRIVATE ws2_32)
endif()
