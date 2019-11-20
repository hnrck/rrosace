# Compilers Compilers configuration, alternatives: gcc, g++, ...
CC ?= clang
CXX ?= clang++

# Name of the binary
BIN ?= rrosace-loop

# Test build dir
BIN_DIR ?= /tmp/

# RROSACE install dir, for env file access
RROSACE_INSTALL_DIR ?= ../../install

include ${RROSACE_INSTALL_DIR}/etc/rrosace/rrosacepathsrc

.PHONY: all

all: ${BIN}

${BIN}: main.c
	${CC} ${^} -o ${BIN_DIR}${@} -lrrosace

run: ${BIN}
	${BIN_DIR}${^}

silent_run: ${BIN}
	${BIN_DIR}${^} 1> /dev/null

check_install: silent_run clean

clean:
	rm ${BIN_DIR}${BIN}
