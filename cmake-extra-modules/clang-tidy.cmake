# Additional target for code linting with clang-tidy.

set(SRCS_DIR ${CMAKE_SOURCE_DIR}/src/)
set(INCS_DIR ${CMAKE_SOURCE_DIR}/include/)
set(TESTS_DIR ${CMAKE_SOURCE_DIR}/test/)
set(EXAMPLES_DIR ${CMAKE_SOURCE_DIR}/examples/)

file(GLOB_RECURSE SRCS ${SRCS_DIR}/*.c)
file(GLOB_RECURSE INCS ${INCS_DIR}/*.h)
file(GLOB_RECURSE C_TESTS ${TESTS_DIR}/*.h ${TESTS_DIR}/*.c)
file(GLOB_RECURSE CXX_TESTS ${TESTS_DIR}/*.cpp)
file(GLOB_RECURSE EXAMPLES ${EXAMPLES_DIR}/*/*.c)

find_program(CLANG-TIDY clang-tidy)

if (CLANG-TIDY)
    add_custom_target(lint)

    set(CLANG_TIDY_CHECKS "*,-clang-analyzer-alpha.*,-llvm-header-guard,-fuchsia-*")
    set(CLANG_TIDY_C_STD c89)
    set(CLANG_TIDY_CXX_STD c++${CMAKE_CXX_STANDARD})

    foreach (SRC ${SRCS})
        add_custom_command(TARGET lint COMMAND ${CLANG-TIDY} -checks=${CLANG_TIDY_CHECKS} ${SRC} -- -std=${CLANG_TIDY_C_STD} -I${INCS_DIR})
    endforeach (SRC ${SRCS})

    foreach (INC ${INCS})
        add_custom_command(TARGET lint COMMAND ${CLANG-TIDY} -checks=${CLANG_TIDY_CHECKS} ${INC} -- -std=${CLANG_TIDY_C_STD} -I${INCS_DIR})
    endforeach (INC ${INCS})

    foreach (TEST ${C_TESTS})
        add_custom_command(TARGET lint COMMAND ${CLANG-TIDY} -checks=${CLANG_TIDY_CHECKS} ${TEST} -- -std=${CLANG_TIDY_C_STD} -I${INCS_DIR})
    endforeach (TEST ${C_TESTS})

    foreach (TEST ${CXX_TESTS})
        add_custom_command(TARGET lint COMMAND ${CLANG-TIDY} -checks=${CLANG_TIDY_CHECKS} ${TEST} -- -std=${CLANG_TIDY_CXX_STD} -I${INCS_DIR} -x c++)
    endforeach (TEST ${CXX_TESTS})

    foreach (EXAMPLE ${EXAMPLES})
      add_custom_command(TARGET lint COMMAND ${CLANG-TIDY} -checks=${CLANG_TIDY_CHECKS} ${EXAMPLE} -- -std=${CLANG_TIDY_C_STD} -I${INCS_DIR})
    endforeach (EXAMPLE ${EXAMPLES})
endif (CLANG-TIDY)
