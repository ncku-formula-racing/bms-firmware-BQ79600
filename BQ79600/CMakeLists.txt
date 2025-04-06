cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME "BQ79600")
project(
  ${PROJECT_NAME}
  LANGUAGES C ASM
)
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    ./
)

file(GLOB SRC_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.c")

target_sources(${PROJECT_NAME} INTERFACE ${SRC_FILES})
