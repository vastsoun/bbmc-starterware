#
# Locating the root directory
#
ROOT=../../../../../../

#
# Device and EVM definitions
#
DEVICE=am335x
EVM=beaglebone

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/build/armv7a/gcc/makedefs

#
# Source files
#
SOURCE=${PLATFORM_SRC}/*.c	

#
# Uncomment one of the following lines to selsect FP support
#
#CFLAGS+=-mfpu=vfpv3 -ftree-vectorize -ffast-math -mfloat-abi=hard
#CFLAGS+=-mfpu=vfp -ftree-vectorize -ffast-math -mfloat-abi=hard
#CFLAGS+=-mfpu=neon -ftree-vectorize -ffast-math -mfloat-abi=hard


#
# Rules for building the source files
#
all: debug release

debug:
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libplatform.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${PLATFORM_BIN}/Debug
	@cp Debug/libplatform.a ${PLATFORM_BIN}/Debug/libplatform.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libplatform.a *.o*
	@mv *.o* Release/
	@mkdir -p ${PLATFORM_BIN}/Release
	@cp Release/libplatform.a ${PLATFORM_BIN}/Release/libplatform.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${PLATFORM_BIN}/Debug ${PLATFORM_BIN}/Release

clean+:
	@rm -rf Debug Release ${PLATFORM_BIN}/Debug ${PLATFORM_BIN}/Release


