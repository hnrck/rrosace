# Additional target for code linting with clang-tidy.

set(SRCS_DIR ${CMAKE_SOURCE_DIR}/src/)
set(INCS_DIR ${CMAKE_SOURCE_DIR}/include/)
set(TESTS_DIR ${CMAKE_SOURCE_DIR}/test/)

file(GLOB_RECURSE SRCS ${SRCS_DIR}/*.c)
file(GLOB_RECURSE INCS ${INCS_DIR}/*.h)
file(GLOB_RECURSE TESTS ${TESTS_DIR}/*.c ${TESTS_DIR}/*.cpp ${TESTS_DIR}/*.h)

find_program(CLANG-FORMAT clang-format)

if (CLANG-FORMAT)
    set(CLANG-FORMAT-STYLE LLVM)
    add_custom_target(format COMMAND ${CLANG-FORMAT} -style=${CLANG-FORMAT-STYLE} -i ${SRCS} ${INCS} ${TESTS})
endif (CLANG-FORMAT)
