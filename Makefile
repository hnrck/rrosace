CC ?= clang
CXX ?= clang++

CXXSTD ?= 98

GENERATOR ?= Ninja

BUILD_TYPE ?= RelWithDebInfo

BUILD_DIR ?= build
INSTALL_DIR ?= install

CPPCHECK ?= 1

all: build

gen:
	cmake -G${GENERATOR} -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_C_COMPILER=${CC} -DCMAKE_CXX_COMPILER=${CXX} -DCMAKE_CXX_STANDARD=${CXXSTD} -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCPPCHECK=1 -H. -B ${BUILD_DIR}

build: gen
	cmake --build ${BUILD_DIR} --target all

install: all
	cmake --build ${BUILD_DIR} --target ${@}

test: all
	cmake --build ${BUILD_DIR} --target ${@}

format:
	cmake --build ${BUILD_DIR} --target ${@}

lint:
	cmake --build ${BUILD_DIR} --target ${@}

clean:
	rm -rf ${BUILD_DIR} ${INSTALL_DIR}
