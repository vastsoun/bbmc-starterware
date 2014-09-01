#
# Locating the root directory
#
ROOT=../../../../../

#
# Device definition (EVM not necessary)
#
DEVICE=am335x

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/build/armv7a/gcc/makedefs

#
# Source files for given target
#
SOURCE= ${SYSCONFIG_SRC}/mmu.c                 \
	${SYSCONFIG_SRC}/cache.c               \
	${SYSCONFIG_SRC}/${COMPILER}/cpu.c     \
	${SYSCONFIG_SRC}/${COMPILER}/cp15.S    \
	${SYSCONFIG_SRC}/${COMPILER}/init.S    \
	${SYSCONFIG_SRC}/${DEVICE}/interrupt.c \
	${SYSCONFIG_SRC}/${DEVICE}/startup.c   \
	${SYSCONFIG_SRC}/${DEVICE}/clock.c     \
	${SYSCONFIG_SRC}/${DEVICE}/${COMPILER}/exceptionhandler.S

#
# Rules for building the source files
#

all: debug release

debug:
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libsystem_config.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${SYSCONFIG_BIN}/Debug
	@cp Debug/libsystem_config.a ${SYSCONFIG_BIN}/Debug/libsystem_config.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libsystem_config.a *.o*
	@mv *.o* Release/
	@mkdir -p ${SYSCONFIG_BIN}/Release
	@cp Release/libsystem_config.a ${SYSCONFIG_BIN}/Release/libsystem_config.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${SYSCONFIG_BIN}/Debug ${SYSCONFIG_BIN}/Release


