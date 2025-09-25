cmake_minimum_required(VERSION 3.15)

# -----------------------------
# Project definition
# -----------------------------
set(PROJECT_NAME go2Reachability)
set(PROJECT_DESCRIPTION ".obj files describing the kinematic constraints of the go2 robot")

project(${PROJECT_NAME} LANGUAGES NONE DESCRIPTION "${PROJECT_DESCRIPTION}")

# -----------------------------
# Source / data files
# -----------------------------
# Assume the .obj files are in the subdirectory data/reachability_constraints
set(REACHABILITY_CONSTRAINTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data/reachability_constraints")

# Collect all .obj files in that directory
file(GLOB OBJ_FILES "${REACHABILITY_CONSTRAINTS_DIR}/*.obj")

# -----------------------------
# Installation
# -----------------------------
# Install the .obj files into data/reachability_constraints relative to CMAKE_INSTALL_PREFIX
install(FILES ${OBJ_FILES}
        DESTINATION "data/reachability_constraints")

# -----------------------------
# Optional: show some info
# -----------------------------
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Description: ${PROJECT_DESCRIPTION}")
message(STATUS "Found OBJ files: ${OBJ_FILES}")
