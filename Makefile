# Compilers Compilers configuration, alternatives: gcc, g++, ...
CC ?= clang
CXX ?= clang++

# CMake generator configuration, alternatives: unix makefiles, ...
GENERATOR ?= Ninja

# CMake build type configuration, alternatives: release, debug,  relwithminsize
BUILD_TYPE ?= RelWithDebInfo

# Directories for building and installing, alternatives: wherever you want
BUILD_DIR ?= build
INSTALL_DIR ?= install

# Activate C++ integration test, alternative: 0 (deactivate)
CPPCHECK ?= 1

# C++ standard for C++ integration test
CXXSTD ?= 98

.PHONY: all gen build install test format lint clean

all: build

# Generate CMake build environment
gen:
	cmake -G${GENERATOR} -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_C_COMPILER=${CC} -DCMAKE_CXX_COMPILER=${CXX} -DCMAKE_CXX_STANDARD=${CXXSTD} -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCPPCHECK=1 -H. -B ${BUILD_DIR}

# Build from CMake build environment
build: gen
	cmake --build ${BUILD_DIR} --target all

# Install built library and headers
install: all
	cmake --build ${BUILD_DIR} --target ${@}

# Run tests
test: all
	cmake --build ${BUILD_DIR} --target ${@}

# Simple loop for illustration
example_loop: all
	cmake --build ${BUILD_DIR} --target ${@}

# Run simple loop
run_example_loop: example_loop
	${BUILD_DIR}/usr/bin/$^

# Format files
format:
	cmake --build ${BUILD_DIR} --target ${@}

# Static code checking
lint:
	cmake --build ${BUILD_DIR} --target ${@}

# Remove build environment
clean:
	rm -rf ${BUILD_DIR}
