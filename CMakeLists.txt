# CMakeLists files in this project can
# refer to the root source directory of the project as ${SOES_SOURCE_DIR} and
# to the root binary directory of the project as ${SOES_BINARY_DIR}.
cmake_minimum_required (VERSION 3.22)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
project (SOES C)

# Warning flags applied per target rather than globally, so that a consumer
# embedding this tree (see docs/cia402-roadmap.md, Phase 5) does not inherit
# -Werror into its own sources.
set (SOES_WARNING_FLAGS -Wall -Wextra -Wconversion -Wno-unused-parameter -Werror)

# Include paths
include_directories(
  ${SOES_SOURCE_DIR}
  ${SOES_SOURCE_DIR}/soes
  )

# Platform flags and sources
include(${CMAKE_SYSTEM_NAME} OPTIONAL)

# Source paths
add_subdirectory (soes)
add_subdirectory (${SOES_DEMO})

message (STATUS "Building for ${CMAKE_SYSTEM_NAME}")
