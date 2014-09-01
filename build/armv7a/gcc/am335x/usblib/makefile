#
# Locating the root directory
#
ROOT=../../../../../

#
# Device definition (required for interrupt API prototypes)
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
SOURCE= ${ROOT}usblib/usbbuffer.c                 \
        ${ROOT}usblib/usbdesc.c                   \
        ${ROOT}usblib/usbringbuf.c                \
        ${ROOT}usblib/usbtick.c                   \
        ${ROOT}usblib/usbdata.c                   \
        ${ROOT}usblib/device/usbdcdc.c            \
        ${ROOT}usblib/device/usbdhid.c            \
        ${ROOT}usblib/device/usbdhidmouse.c       \
        ${ROOT}usblib/device/usbdmsc.c            \
        ${ROOT}usblib/device/usbdbulk.c           \
        ${ROOT}usblib/device/usbdcdesc.c          \
        ${ROOT}usblib/device/usbdcomp.c           \
        ${ROOT}usblib/device/usbdconfig.c         \
        ${ROOT}usblib/device/usbdenum.c           \
        ${ROOT}usblib/device/usbdhandler.c        \
        ${ROOT}usblib/host/usbhostenum.c          \
        ${ROOT}usblib/host/usbhhid.c              \
        ${ROOT}usblib/host/usbhmsc.c              \
        ${ROOT}usblib/host/usbhscsi.c             \
        ${ROOT}usblib/host/usbhhidmouse.c

#
# Rules for building the source files
#
all: debug release

debug:
	mkdir -p Debug 
	$(CC) $(CFLAGS) -g -D DMA_MODE $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libusblib.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${USBLIB_BIN}/Debug
	@cp Debug/libusblib.a ${USBLIB_BIN}/Debug/libusblib.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 -D DMA_MODE $(SOURCE)
	$(AR) $(ARFLAGS) Release/libusblib.a *.o*
	@mv *.o* Release/
	@mkdir -p ${USBLIB_BIN}/Release
	@cp Release/libusblib.a ${USBLIB_BIN}/Release/libusblib.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${USBLIB_BIN}/Debug ${USBLIB_BIN}/Release



