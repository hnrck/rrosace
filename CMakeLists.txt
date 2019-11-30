# file CMakeLists.txt
# author Henrick Deschamps
# version 1.1.0
# date 2016-06-10

#-----------------------------------------------------------------------------------------------------------------------
# Project configuration

# Project name
project(rrosace C CXX)

# CMake minimum version required set by default, might be changed.
cmake_minimum_required(VERSION 3.8)

# Description summary of the project.
set(DESCRIPTION_SUMMARY "RROSACE")
set(DESCRIPTION_SUMMARY "${DESCRIPTION_SUMMARY} ---")
set(DESCRIPTION_SUMMARY "${DESCRIPTION_SUMMARY} Redundant-ROSACE")

# Application Binary Interface version for library/application interfacing.
set(ABI_VERSION_MAJOR "1")
set(ABI_VERSION_MINOR "1")
set(ABI_VERSION_PATCH "0")
set(ABI_VERSION "${ABI_VERSION_MAJOR}.${ABI_VERSION_MINOR}.${ABI_VERSION_PATCH}")

# Setting compilation options.
set(CMAKE_C_STANDARD 90)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpedantic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")

set(CMAKE_CXX_STANDARD CACHE STRING 98)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++")

# c flags depending on the build type.
set(cmake_c_flags_debug "${cmake_c_flags_debug} -o0 -g")
set(cmake_c_flags_release "${cmake_c_flags_release} -o3 -dndebug")
set(cmake_c_flags_relwithdebinfo "${cmake_c_flags_relwithdebinfo} -o2 -g")
set(cmake_c_flags_minsizerel "${cmake_c_flags_minsizerel} -os")

# cxx flags depending on the build type.
set(cmake_cxx_flags_debug "${cmake_cxx_flags_debug} -o0 -g")
set(cmake_cxx_flags_release "${cmake_cxx_flags_release} -o3 -dndebug")
set(cmake_cxx_flags_relwithdebinfo "${cmake_cxx_flags_relwithdebinfo} -o2 -g")
set(cmake_cxx_flags_minsizerel "${cmake_cxx_flags_minsizerel} -os")

# Default build type if none provided.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif ()

# Location for the rrosace library build.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY usr/lib)
set(CMAKE_INCLUDE_OUTPUT_DIRECTORY usr/include)
#-----------------------------------------------------------------------------------------------------------------------

#-----------------------------------------------------------------------------------------------------------------------
# Extra modules

# Allowing alternatives for compilations.
option(COVERAGE "Allow code coverage. (requires GCOV. Optionnaly LCOV and genhtml for reports)." OFF)

# Adding CMake extra modules for code linting/formatting.
set(CMAKE_EXTRA_MODULES_DIR ${CMAKE_SOURCE_DIR}/cmake-extra-modules)
set(CMAKE_EXTRA_MODULES clang-format clang-tidy)
foreach (CMAKE_EXTRA_MODULE ${CMAKE_EXTRA_MODULES})
    include(${CMAKE_EXTRA_MODULES_DIR}/${CMAKE_EXTRA_MODULE}.cmake)
endforeach ()
#-----------------------------------------------------------------------------------------------------------------------

#-----------------------------------------------------------------------------------------------------------------------
# Build

# Include directories
include_directories(
        ${CMAKE_SOURCE_DIR}/include
)

set(SRC_RROSACE
        ${CMAKE_SOURCE_DIR}/src/engine.c
        ${CMAKE_SOURCE_DIR}/src/elevator.c
        ${CMAKE_SOURCE_DIR}/src/flight_dynamics.c
        ${CMAKE_SOURCE_DIR}/src/filters.c
        ${CMAKE_SOURCE_DIR}/src/fcu.c
        ${CMAKE_SOURCE_DIR}/src/flight_mode.c
        ${CMAKE_SOURCE_DIR}/src/fcc.c
        ${CMAKE_SOURCE_DIR}/src/cables.c)

if (APPLE)
    set(CMAKE_MACOSX_RPATH ON)
    set(CMAKE_INSTALL_RPATH "")
endif ()

add_library(${PROJECT_NAME} SHARED ${SRC_RROSACE})
target_link_libraries(${PROJECT_NAME} m)

set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${ABI_VERSION_MAJOR} VERSION ${ABI_VERSION})
#-----------------------------------------------------------------------------------------------------------------------


#-----------------------------------------------------------------------------------------------------------------------
# Test

enable_testing()

add_library(test_common STATIC ${CMAKE_SOURCE_DIR}/test/test_common.c)

function(module_test MODULE_NAME)
    set(TEST_${MODULE_NAME} ${PROJECT_NAME}_${MODULE_NAME}_test)
    set(SRC_RROSACE${MODULE_NAME}_TEST ${CMAKE_SOURCE_DIR}/test/${MODULE_NAME}_test.c)
    add_executable(${TEST_${MODULE_NAME}} ${SRC_RROSACE${MODULE_NAME}_TEST})
    target_link_libraries(${TEST_${MODULE_NAME}} ${PROJECT_NAME} test_common)
    add_test(${TEST_${MODULE_NAME}} ${CMAKE_BINARY_DIR}/${TEST_${MODULE_NAME}})
endfunction()

module_test(engine)
module_test(elevator)
module_test(flight_dynamics)
module_test(filters)
module_test(fcu)
module_test(flight_mode)
module_test(fcc)
module_test(cables)

if (CPPCHECK)
    set(TEST_CPP_PORT ${PROJECT_NAME}_cpp_test)
    set(SRC_RROSACE_CPP_PORT_TEST ${CMAKE_SOURCE_DIR}/test/cpp_test.cpp)
    add_executable(${TEST_CPP_PORT} ${SRC_RROSACE_CPP_PORT_TEST})
    target_link_libraries(${TEST_CPP_PORT} ${PROJECT_NAME})
    add_test(${TEST_CPP_PORT} ${CMAKE_BINARY_DIR}/${TEST_CPP_PORT})
endif (CPPCHECK)

if (MEMCHECK)
    include(Dart)
    set(MEMCHECK_COMMAND test -O memcheck.log -D ExperimentalMemCheck)
    add_custom_target(memcheck COMMAND ctest -O ${MEMCHECK_COMMAND})
endif (MEMCHECK)

#-----------------------------------------------------------------------------------------------------------------------


#-----------------------------------------------------------------------------------------------------------------------
# Examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY usr/bin)

# Simple simulation loop
add_executable(example_loop ${CMAKE_SOURCE_DIR}/examples/loop/main.c)
target_link_libraries(example_loop rrosace)
set_target_properties(example_loop PROPERTIES SOVERSION ${ABI_VERSION_MAJOR} VERSION ${ABI_VERSION})

# Simple simulation loop in C++
add_executable(example_loop_cpp ${CMAKE_SOURCE_DIR}/examples/loop_cpp/main.cpp)
target_link_libraries(example_loop_cpp rrosace)
set_target_properties(example_loop_cpp PROPERTIES SOVERSION ${ABI_VERSION_MAJOR} VERSION ${ABI_VERSION})

#-----------------------------------------------------------------------------------------------------------------------


#-----------------------------------------------------------------------------------------------------------------------
# Installation

# Include file to install
set(RROSACE_INC
        ${CMAKE_SOURCE_DIR}/include/rrosace.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_engine.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_elevator.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_flight_dynamics.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_filters.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_fcu.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_flight_mode.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_fcc.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_cables.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_constants.h
        ${CMAKE_SOURCE_DIR}/include/rrosace_common.h
        )

# Path configuration for SEAPlanes.
set(RROSACE_PATHS_CONF rrosacepathsrc)
file(WRITE ${CMAKE_BINARY_DIR}/${RROSACE_PATHS_CONF}
        "export LIBRARY_PATH=$LIBRARY_PATH:${CMAKE_INSTALL_PREFIX}/${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${PROJECT_NAME}/\n"
        "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${CMAKE_INSTALL_PREFIX}/${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${PROJECT_NAME}/\n"
        "export C_PATH=$C_PATH:${CMAKE_INSTALL_PREFIX}/${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/${PROJECT_NAME}/\n"
        "export C_INCLUDE_PATH=$C_INCLUDE_PATH:${CMAKE_INSTALL_PREFIX}/${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/${PROJECT_NAME}/\n"
        "export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:${CMAKE_INSTALL_PREFIX}/${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/${PROJECT_NAME}/\n"
        )
if (APPLE)
    file(APPEND ${CMAKE_BINARY_DIR}/${RROSACE_PATHS_CONF} "export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:${CMAKE_INSTALL_PREFIX}/${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${PROJECT_NAME}/\n")
endif ()

# Configurations to install.
set(RROSACE_CONFS
        ${CMAKE_BINARY_DIR}/${RROSACE_PATHS_CONF}
        )

# Installation recipes.
# The SEAPlanes library.
set(LIBRARY_INSTALL_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${PROJECT_NAME}/)
install(TARGETS ${PROJECT_NAME} PUBLIC_HEADER LIBRARY DESTINATION ${LIBRARY_INSTALL_DIR})

# The SEAPlanes library includes.
set(INCLUDE_INSTALL_DIR ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/${PROJECT_NAME}/)
install(FILES ${RROSACE_INC} DESTINATION ${INCLUDE_INSTALL_DIR})

# The SEAPlanes library configuration files.
set(CONFS_INSTALL_DIR etc/${PROJECT_NAME})
install(FILES ${RROSACE_CONFS} DESTINATION ${CONFS_INSTALL_DIR}/)


# If in debug, coverage report can be generated with <cmake> ... -DCOVERAGE=ON and <make> coverage
if (CMAKE_BUILD_TYPE MATCHES "Debug")
    if (COVERAGE)
        find_program(MAKE_DIR mkdir)
        find_program(LCOV lcov)
        find_program(GEN_HTML genhtml)
        find_program(PRINT echo)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
        if ("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
            set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS "--coverage ")
        endif ()
        target_link_libraries(${PROJECT_NAME} gcov)
        set(COVERAGE_DIR ${CMAKE_BINARY_DIR}/coverage)
        set(CMAKE_COV
                ${MAKE_DIR} -p ${COVERAGE_DIR}
                && ${LCOV} -c
                -b ${CMAKE_BINARY_DIR}/CMakeFiles/${PROJECT_NAME}.dir/src
                -d ${CMAKE_BINARY_DIR}/CMakeFiles/${PROJECT_NAME}.dir/src
                -o ${COVERAGE_DIR}/coverage.run
                && ${GEN_HTML} -o ${COVERAGE_DIR} ${COVERAGE_DIR}/coverage.run
                && ${PRINT} "Coverage report generated. Please open ${COVERAGE_DIR}/index.html with a browser.")
        add_custom_target(coverage COMMAND ${CMAKE_COV})
    endif ()
else ()
    if (COVERAGE)
        message(WARNING
                "coverage is only available with Debug build type."
                )
    endif ()
endif ()

# Special install_rrosace target for certain IDE.
add_custom_target(install_${PROJECT_NAME}
        "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}" --target install
        DEPENDS ${PROJECT_NAME}
        COMMENT "Installing ${PROJECT_NAME} in ${CMAKE_INSTALL_PREFIX}"
        )
#-----------------------------------------------------------------------------------------------------------------------

#-----------------------------------------------------------------------------------------------------------------------
# Documentation, with Doxygen

find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

if (BUILD_DOCUMENTATION)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
endif ()
#-----------------------------------------------------------------------------------------------------------------------

#-----------------------------------------------------------------------------------------------------------------------
# Project summary for CMake run

string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

message(STATUS "+------------------------------------")
message(STATUS "|   ${PROJECT_NAME} built summary")
message(STATUS "+------------------------------------")
message(STATUS "| Description:")
message(STATUS "| ${DESCRIPTION_SUMMARY}")
message(STATUS "+------------------------------------")
message(STATUS "| ")
message(STATUS "| library ABI version: ${ABI_VERSION}")
message(STATUS "| generator:           ${CMAKE_GENERATOR}")
message(STATUS "| build type:          ${CMAKE_BUILD_TYPE}")
message(STATUS "| ")
message(STATUS "| C:                   ${CMAKE_C_COMPILER}")
message(STATUS "| STD:                 ${CMAKE_C_STANDARD}")
message(STATUS "| CFLAGS:              ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${BUILD_TYPE}}")
message(STATUS "| CXX:                 ${CMAKE_CXX_COMPILER}")
message(STATUS "| STD:                 ${CMAKE_CXX_STANDARD}")
message(STATUS "| CXXFLAGS:            ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${BUILD_TYPE}}")
message(STATUS "| LDFLAGS:             ${LDFLAGS}")
message(STATUS "| ")
message(STATUS "| source directory:    ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "| build directory:     ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "| install prefix:      ${CMAKE_INSTALL_PREFIX}")
message(STATUS "| include directory:   ${CMAKE_INSTALL_PREFIX}/${INCLUDE_INSTALL_DIR}")
message(STATUS "| library directory:   ${CMAKE_INSTALL_PREFIX}/${LIBRARY_INSTALL_DIR}")
message(STATUS "| confs directory:     ${CMAKE_INSTALL_PREFIX}/${CONFS_INSTALL_DIR}")
message(STATUS "| ")
if (CMAKE_BUILD_TYPE MATCHES "Debug")
    message(STATUS "| coverage:            ${COVERAGE}")
endif ()
message(STATUS "+------------------------------------")
#-----------------------------------------------------------------------------------------------------------------------
