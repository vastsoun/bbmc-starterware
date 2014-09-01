# Locating the root directory
#
ROOT=../../../../

DEVICE=am335x
#
# Include the makefile definitions. This contains compiler, linker and
# archiver definitions and options
#
include ${ROOT}/build/armv7a/gcc/makedefs

SOURCE=$(ROOT)/contrlib/*.c

#
# Rules for building the source files
#
all: debug release

debug:
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libcontrl.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${CONTROL_BIN}/Debug
	@cp Debug/libcontrl.a ${CONTROL_BIN}/Debug/libcontrl.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libcontrl.a *.o*
	@mv *.o* Release/
	@mkdir -p ${CONTROL_BIN}/Release
	@cp Release/libcontrl.a ${CONTROL_BIN}/Release/libcontrl.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${CONTROL_BIN}/Debug ${CONTROL_BIN}/Release

