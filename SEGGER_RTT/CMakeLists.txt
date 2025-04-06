cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME "segger_rtt")
project(
  ${PROJECT_NAME}
  LANGUAGES C ASM
)
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    ./Config
    ./RTT
    ./Syscalls
)

file(GLOB SRC_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "RTT/*.c")

target_sources(${PROJECT_NAME} INTERFACE ${SRC_FILES})
