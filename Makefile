#
#             LUFA Library
#     Copyright (C) Dean Camera, 2012.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

MCU          = atmega32u4
ARCH         = AVR8
BOARD        = NONE
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = MIDI
SRC          = main.c \
							 spi.c \
               xnormidi/midi.c \
               xnormidi/midi_device.c \
               xnormidi/bytequeue/bytequeue.c \
               xnormidi/bytequeue/interrupt_setting.c \
							 xnormidi/implementations/lufa_midi/midi_usb.c \
               xnormidi/implementations/lufa_midi/Descriptors.c \
               $(LUFA_SRC_USB)                     \
               $(LUFA_SRC_USBCLASS)
LUFA_PATH    = ../LUFA-120730/LUFA/
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -I. -Ixnormidi/ -Ixnormidi/implementations/lufa_midi/ -Ixnormidi/implementations/lufa_midi/Config/
LD_FLAGS     =

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

program: $(TARGET).hex $(TARGET).eep
	@teensy_loader_cli -mmcu=$(MCU) -p -w -v $(TARGET).hex
