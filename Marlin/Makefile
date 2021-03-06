# Sprinter Arduino Project Makefile
#
# Makefile Based on:
# Arduino 0011 Makefile
# Arduino adaptation by mellis, eighthave, oli.keller
# Marlin adaption by Daid
#
# This has been tested with Arduino 1.0.5.
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
# Detailed instructions for using the makefile:
#
#  1. Modify the line containg "ARDUINO_INSTALL_DIR" to point to the directory that
#     contains the Arduino installation (for example, under Mac OS X, this
#     might be /Applications/arduino-0012).
#
#  2. Modify the line containing "UPLOAD_PORT" to refer to the filename
#     representing the USB or serial connection to your Arduino board
#     (e.g. UPLOAD_PORT = /dev/tty.USB0).  If the exact name of this file
#     changes, you can use * as a wildcard (e.g. UPLOAD_PORT = /dev/tty.usb*).
#
#  3. Set the line containing "MCU" to match your board's processor.
#     Older one's are atmega8 based, newer ones like Arduino Mini, Bluetooth
#     or Diecimila have the atmega168.  If you're using a LilyPad Arduino,
#     change F_CPU to 8000000. If you are using Gen7 electronics, you
#     probably need to use 20000000. Either way, you must regenerate
#     the speed lookup table with create_speed_lookuptable.py.
#
#  4. Type "make" and press enter to compile/verify your program.
#
#  5. Type "make upload", reset your Arduino board, and press enter to
#     upload your program to the Arduino board.
#
# Note that all settings are set with ?=, this means you can override them
# from the commandline with "make HARDWARE_MOTHERBOARD=71" for example

# This defined the board you are compiling for (see Configuration.h for the options)
HARDWARE_MOTHERBOARD ?= 72

# Arduino source install directory, and version number
ARDUINO_INSTALL_DIR  ?= /home/johan/Dev/arduino-1.5.8/
ARDUINO_VERSION      ?= 158

# You can optionally set a path to the avr-gcc tools. Requires a trailing slash. (ex: /usr/local/avr-gcc/bin)
AVR_TOOLS_PATH ?= $(ARDUINO_INSTALL_DIR)/hardware/tools/avr/bin/

#Programmer configuration
UPLOAD_RATE        ?= 115200
AVRDUDE_PROGRAMMER ?= wiring
UPLOAD_PORT        ?= /dev/arduino

#Directory used to build files in, contains all the build files, from object files to the final hex file.
BUILD_DIR          ?= applet

# This defines whether Liquid_TWI2 support will be built
LIQUID_TWI2 ?= 0

############################################################################
# Below here nothing should be changed...

# Here the Arduino variant is selected by the board type
# HARDWARE_VARIANT = "arduino", "Sanguino", "Gen7", ...
# MCU = "atmega1280", "Mega2560", "atmega2560", "atmega644p", ...

#Gen7
ifeq ($(HARDWARE_MOTHERBOARD),10)
HARDWARE_VARIANT ?= Gen7
MCU              ?= atmega644
F_CPU            ?= 20000000
else ifeq  ($(HARDWARE_MOTHERBOARD),11)
HARDWARE_VARIANT ?= Gen7
MCU              ?= atmega644p
F_CPU            ?= 20000000
else ifeq  ($(HARDWARE_MOTHERBOARD),12)
HARDWARE_VARIANT ?= Gen7
MCU              ?= atmega644p
F_CPU            ?= 20000000
else ifeq  ($(HARDWARE_MOTHERBOARD),13)
HARDWARE_VARIANT ?= Gen7
MCU              ?= atmega1284p
F_CPU            ?= 20000000

#RAMPS
else ifeq  ($(HARDWARE_MOTHERBOARD),3)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega2560
else ifeq  ($(HARDWARE_MOTHERBOARD),33)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega2560
else ifeq  ($(HARDWARE_MOTHERBOARD),34)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega2560

#Duemilanove w/ ATMega328P pin assignment
else ifeq  ($(HARDWARE_MOTHERBOARD),4)
HARDWARE_VARIANT ?= arduino
HARDWARE_SUB_VARIANT ?= standard
MCU              ?= atmega328p

#Gen6
else ifeq  ($(HARDWARE_MOTHERBOARD),5)
HARDWARE_VARIANT ?= Gen6
MCU              ?= atmega644p
else ifeq  ($(HARDWARE_MOTHERBOARD),51)
HARDWARE_VARIANT ?= Gen6
MCU              ?= atmega644p

#Sanguinololu
else ifeq  ($(HARDWARE_MOTHERBOARD),6)
HARDWARE_VARIANT ?= Sanguino
MCU              ?= atmega644p
else ifeq  ($(HARDWARE_MOTHERBOARD),62)
HARDWARE_VARIANT ?= Sanguino
MCU              ?= atmega644p
else ifeq  ($(HARDWARE_MOTHERBOARD),63)
HARDWARE_VARIANT ?= Sanguino
MCU              ?= atmega644p

#Ultimaker
else ifeq  ($(HARDWARE_MOTHERBOARD),7)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega2560
else ifeq  ($(HARDWARE_MOTHERBOARD),71)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega1280
else ifeq  ($(HARDWARE_MOTHERBOARD),72)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega2560

#Teensylu
else ifeq  ($(HARDWARE_MOTHERBOARD),8)
HARDWARE_VARIANT ?= Teensy
MCU              ?= at90usb1286
else ifeq  ($(HARDWARE_MOTHERBOARD),81)
HARDWARE_VARIANT ?= Teensy
MCU              ?= at90usb1286
else ifeq  ($(HARDWARE_MOTHERBOARD),82)
HARDWARE_VARIANT ?= Teensy
MCU              ?= at90usb646

#Gen3+
else ifeq  ($(HARDWARE_MOTHERBOARD),9)
HARDWARE_VARIANT ?= Sanguino
MCU              ?= atmega644p

#Megatronics
else ifeq  ($(HARDWARE_MOTHERBOARD),70)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega2560

#Alpha OMCA board
else ifeq  ($(HARDWARE_MOTHERBOARD),90)
HARDWARE_VARIANT ?= SanguinoA
MCU              ?= atmega644

#Final OMCA board
else ifeq  ($(HARDWARE_MOTHERBOARD),91)
HARDWARE_VARIANT ?= Sanguino
MCU              ?= atmega644p

#Rambo
else ifeq  ($(HARDWARE_MOTHERBOARD),301)
HARDWARE_VARIANT ?= arduino
MCU              ?= atmega2560

endif

# Be sure to regenerate speed_lookuptable.h with create_speed_lookuptable.py
# if you are setting this to something other than 16MHz
# Set to 16Mhz if not yet set.
F_CPU ?= 16000000

# Arduino containd the main source code for the Arduino
# Libraries, the "hardware variant" are for boards
# that derives from that, and their source are present in
# the main Marlin source directory
ifeq ($(HARDWARE_VARIANT), arduino)
	ifeq ($(shell [ $(ARDUINO_VERSION) -ge 150 ] && echo true ), true)
		HARDWARE_DIR = $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr
		HARDWARE_SRC = $(HARDWARE_DIR)/cores/arduino
	else
		HARDWARE_DIR = $(ARDUINO_INSTALL_DIR)/hardware
	endif
	HARDWARE_LIB_DIR = $(ARDUINO_INSTALL_DIR)/libraries
else ifeq ($(HARDWARE_VARIANT), arduino_due)
	HARDWARE_DIR = $(ARDUINO_INSTALL_DIR)/hardware/arduino/sam
	HARDWARE_SRC ?= $(HARDWARE_DIR)/cores/arduino
	HARDWARE_LIB_DIR = $(ARDUINO_INSTALL_DIR)/libraries/
else
	ifeq ($(shell [ $(ARDUINO_VERSION) -ge 100 ] && echo true), true)
		HARDWARE_DIR = ../ArduinoAddons/Arduino_1.x.x
	else
		HARDWARE_DIR = ../ArduinoAddons/Arduino_0.xx
	endif
endif
HARDWARE_SRC ?= $(HARDWARE_DIR)/$(HARDWARE_VARIANT)/cores/arduino
HARDWARE_LIB_DIR ?= $(HARDWARE_DIR)/libraries

TARGET = $(notdir $(CURDIR))

# VPATH tells make to look into these directory for source files,
# there is no need to specify explicit pathnames as long as the
# directory is added here

VPATH = .
VPATH += $(BUILD_DIR)
VPATH += $(HARDWARE_SRC)

ifeq ($(LIQUID_TWI2), 1)
	VPATH += $(HARDWARE_LIB_DIR)/Wire
	VPATH += $(HARDWARE_LIB_DIR)/Wire/utilities
	VPATH += $(HARDWARE_LIB_DIR)/libraries/LiquidTWI2
endif

ifeq ($(HARDWARE_VARIANT), arduino)
	HARDWARE_SUB_VARIANT ?= mega
	ifeq ($(shell [ $(ARDUINO_VERSION) -ge 158 ] && echo true), true)
		DEFINES += __PROG_TYPES_COMPAT__
		VPATH += $(HARDWARE_DIR)/variants/$(HARDWARE_SUB_VARIANT)
		VPATH += $(HARDWARE_DIR)/libraries/SPI
		VPATH += $(HARDWARE_LIB_DIR)/LiquidCrystal/src
	else ifeq ($(shell [ $(ARDUINO_VERSION) -ge 150 ] && echo true), true)
		DEFINES += __PROG_TYPES_COMPAT__
		VPATH += $(HARDWARE_DIR)/variants/$(HARDWARE_SUB_VARIANT)
		VPATH += $(HARDWARE_LIB_DIR)/SPI/arch/avr
		VPATH += $(HARDWARE_LIB_DIR)/LiquidCrystal/src
	else
		VPATH += $(HARDWARE_DIR)/$(HARDWARE_VARIANT)/variants/$(HARDWARE_SUB_VARIANT)
		VPATH += $(HARDWARE_LIB_DIR)/SPI
		VPATH += $(HARDWARE_LIB_DIR)/LiquidCrystal
	endif
else ifeq ($(HARDWARE_VARIANT), arduino_due)
	HARDWARE_SUB_VARIANT ?= arduino_due_x
	VPATH += $(HARDWARE_DIR)/variants/$(HARDWARE_SUB_VARIANT)
	VPATH += $(HARDWARE_LIB_DIR)/SPI/arch/sam
	VPATH += $(HARDWARE_LIB_DIR)/LiquidCrystal/src
	VPATH += $(HARDWARE_DIR)/system/libsam
	VPATH += $(HARDWARE_DIR)/system/CMSIS/Device/ATMEL/
	VPATH += $(HARDWARE_DIR)/system/CMSIS/CMSIS/Include/
	DEFINES += ARDUINO_SAM_DUE ARDUINO_ARCH_SAM __SAM_3X8E__ USB_VID=0x2341 USB_PID=0x003e AT90USB
else
	HARDWARE_SUB_VARIANT ?= standard
	VPATH += $(HARDWARE_DIR)/$(HARDWARE_VARIANT)/variants/$(HARDWARE_SUB_VARIANT)
	VPATH += $(HARDWARE_LIB_DIR)/SPI
	VPATH += $(HARDWARE_LIB_DIR)/LiquidCrystal
endif

SRC = wiring.c \
	wiring_analog.c wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c WInterrupts.c
CXXSRC = WMath.cpp WString.cpp Print.cpp Marlin_main.cpp	\
	MarlinSerial.cpp Sd2Card.cpp SdBaseFile.cpp SdFatUtil.cpp	\
	SdFile.cpp SdVolume.cpp motion_control.cpp planner.cpp		\
	stepper.cpp temperature.cpp cardreader.cpp ConfigurationStore.cpp \
	watchdog.cpp electronics_test.cpp \
	Sd2AsyncCard.cpp SdAsyncVolume.cpp SdAsyncBaseFile.cpp SdAsyncFile.cpp
CXXSRC += LiquidCrystal.cpp ultralcd.cpp SPI.cpp Servo.cpp Tone.cpp
CXXSRC += UltiLCD2.cpp UltiLCD2_gfx.cpp UltiLCD2_hi_lib.cpp UltiLCD2_low_lib.cpp \
	UltiLCD2_menu_first_run.cpp UltiLCD2_menu_maintenance.cpp UltiLCD2_menu_material.cpp \
	UltiLCD2_menu_print.cpp lifetime_stats.cpp

ifeq ( $(HARDWARE_VARIANT), arduino)
	ifeq ($(shell [ $(ARDUINO_VERSION) -ge 150 ] && echo true), true)
		CXXSRC += hooks.cpp
	endif
else ifeq ($(HARDWARE_VARIANT), arduino_due)
		CXXSRC += hooks.cpp
endif

ifneq ($(LIQUID_TWI2), 0)
	SRC += twi.c
	CPPSRC += Wire.cpp LiquidTWI2.cpp
endif

#Check for Arduino 1.0.0 or higher and use the correct sourcefiles for that version
ifeq ($(shell [ $(ARDUINO_VERSION) -ge 100 ] && echo true), true)
CXXSRC += main.cpp
else
SRC += pins_arduino.c main.c
endif

FORMAT = ihex

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
ifneq ($(HARDWARE_VARIANT), arduino_due)
	DEBUG = stabs
endif

OPT = s

DEFINES ?=

# Program settings
ifeq ($(HARDWARE_VARIANT), arduino_due)
CC = $(ARM_TOOLS_PATH)arm-none-eabi-gcc
CXX = $(ARM_TOOLS_PATH)arm-none-eabi-g++
OBJCOPY = $(ARM_TOOLS_PATH)arm-none-eabi-objcopy
OBJDUMP = $(ARM_TOOLS_PATH)arm-none-eabi-objdump
AR  = $(ARM_TOOLS_PATH)arm-none-eabi-ar
SIZE = $(ARM_TOOLS_PATH)arm-none-eabi-size
NM = $(ARM_TOOLS_PATH)arm-none-eabi-nm
else
CC = $(AVR_TOOLS_PATH)avr-gcc
CXX = $(AVR_TOOLS_PATH)avr-g++
OBJCOPY = $(AVR_TOOLS_PATH)avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)avr-objdump
AR  = $(AVR_TOOLS_PATH)avr-ar
SIZE = $(AVR_TOOLS_PATH)avr-size
NM = $(AVR_TOOLS_PATH)avr-nm
endif
AVRDUDE = avrdude
REMOVE = rm -f
MV = mv -f

# Place -D or -U options here
CDEFS    = -DF_CPU=$(F_CPU) ${addprefix -D , $(DEFINES)}
CXXDEFS  = $(CDEFS)

ifeq ($(HARDWARE_VARIANT), Teensy)
CDEFS  += -DUSB_SERIAL
SRC    += usb.c pins_teensy.c
CXXSRC += usb_api.cpp
endif

# Add all the source directories as include directories too
CINCS = ${addprefix -I ,${VPATH}}
CXXINCS = ${addprefix -I ,${VPATH}}

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
#CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG)
CWARN = -Wall -Wstrict-prototypes
CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct \
	-fshort-enums -w -ffunction-sections -fdata-sections \
	-DARDUINO=$(ARDUINO_VERSION)
ifneq ($(HARDWARE_MOTHERBOARD),)
CTUNING += -DMOTHERBOARD=${HARDWARE_MOTHERBOARD}
endif
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)

CFLAGS := $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CEXTRA) $(CTUNING)
CXXFLAGS :=         $(CDEFS) $(CINCS) -O$(OPT) -Wall    $(CEXTRA) $(CTUNING)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs
LDFLAGS = -lm


# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(UPLOAD_PORT)
AVRDUDE_WRITE_FLASH = -U flash:w:$(BUILD_DIR)/$(TARGET).hex:i
AVRDUDE_FLAGS = -D -C $(ARDUINO_INSTALL_DIR)/hardware/tools/avrdude.conf \
	-p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) \
	-b $(UPLOAD_RATE)

# Define all object files.
OBJ = ${patsubst %.c, $(BUILD_DIR)/%.o, ${SRC}}
OBJ += ${patsubst %.cpp, $(BUILD_DIR)/%.o, ${CXXSRC}}
OBJ += ${patsubst %.S, $(BUILD_DIR)/%.o, ${ASRC}}

# Define all listing files.
LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.

ifeq ($(HARDWARE_VARIANT), arduino_due)
	MCU_FLAG = -mcpu=cortex-m3 -mthumb
else
	MCU_FLAG=-mmcu=$(MCU)
endif

ALL_CFLAGS = $(MCU_FLAG) -I. $(CFLAGS)
ALL_CXXFLAGS = $(MCU_FLAG) $(CXXFLAGS)
ALL_ASFLAGS = $(MCU_FLAG) -x assembler-with-cpp $(ASFLAGS)

# set V=1 (eg, "make V=1") to print the full commands etc.
ifneq ($V,1)
 Pecho=@echo
 P=@
else
 Pecho=@:
 P=
endif

# Default target.
all: sizeafter

build: $(BUILD_DIR) elf hex

# Creates the object directory
$(BUILD_DIR):
	$P mkdir -p $(BUILD_DIR)

elf: $(BUILD_DIR)/$(TARGET).elf
hex: $(BUILD_DIR)/$(TARGET).hex
eep: $(BUILD_DIR)/$(TARGET).eep
lss: $(BUILD_DIR)/$(TARGET).lss
sym: $(BUILD_DIR)/$(TARGET).sym

# Program the device.
# Do not try to reset an arduino if it's not one
upload: $(BUILD_DIR)/$(TARGET).hex
ifeq (${AVRDUDE_PROGRAMMER}, arduino)
	stty hup < $(UPLOAD_PORT); true
endif
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)
ifeq (${AVRDUDE_PROGRAMMER}, arduino)
	stty -hup < $(UPLOAD_PORT); true
endif

	# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(BUILD_DIR)/$(TARGET).hex
ELFSIZE = $(SIZE) --mcu=$(MCU) -C $(BUILD_DIR)/$(TARGET).elf; \
          $(SIZE)  $(BUILD_DIR)/$(TARGET).elf
sizebefore:
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter: build
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
	--change-section-address .data-0x800000 \
	--change-section-address .bss-0x800000 \
	--change-section-address .noinit-0x800000 \
	--change-section-address .eeprom-0x810000


coff: $(BUILD_DIR)/$(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(BUILD_DIR)/$(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(BUILD_DIR)/$(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym
.PRECIOUS: .o

.elf.hex:
	$(Pecho) "  COPY  $@"
	$P $(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
		--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

	# Link: create ELF output file from library.
$(BUILD_DIR)/$(TARGET).elf: $(OBJ) Configuration.h
	$(Pecho) "  CXX   $@"
	$P $(CC) $(ALL_CXXFLAGS) -Wl,--gc-sections -o $@ -L. $(OBJ) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CC    $<"
	$P $(CC) -MMD -c $(ALL_CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(BUILD_DIR)/%.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS) $< -o $@


# Target: clean project.
clean:
	$(Pecho) "  RM    $(BUILD_DIR)/*"
	$P $(REMOVE) $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).eep $(BUILD_DIR)/$(TARGET).cof $(BUILD_DIR)/$(TARGET).elf \
		$(BUILD_DIR)/$(TARGET).map $(BUILD_DIR)/$(TARGET).sym $(BUILD_DIR)/$(TARGET).lss $(BUILD_DIR)/$(TARGET).cpp \
		$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d)
	$(Pecho) "  RMDIR $(BUILD_DIR)/"
	$P rm -rf $(BUILD_DIR)


.PHONY:	all build elf hex eep lss sym program coff extcoff clean depend sizebefore sizeafter

# Automaticaly include the dependency files created by gcc
-include ${wildcard $(BUILD_DIR)/*.d}
