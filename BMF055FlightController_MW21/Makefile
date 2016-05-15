###########################################################################
# 	The MIT License (MIT)
#
#	Copyright (c) 2016
#
#	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
#	to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
#	and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
#	INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
#	IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
#	DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
#	ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
#############################################################################################################

#############################################################################
# APPLICATION NAME (Binary Name)
USERAPP_NAME = BMF055FlightController

####################################################################
# Makefile                                                         #
####################################################################
.SUFFIXES:	# ignore build in rules
.PHONY: all debug release clean flash

################# Include Makefiles		 ###########################
include ./Make_hwPfDef.mk	# hardware specific makefile
include ./Makefile_ASF.mk	# includes sources and includes folder for ASF
include ./Makefile_APP.mk	# application specific  

#####################################################################
# Tools directories, for Compilation, Debug and Code Flashing		#
#####################################################################
TOOLDIR = C:/Program Files (x86)
JLINKARMDIR = $(TOOLDIR)/SEGGER/JLink_V510p
DOXYGEN_DIR = $(TOOLDIR)/doxygen/V1.8.1.1_32Bit
GRAPHVIZ_DIR = $(TOOLDIR)/graphviz-2.34/release/bin

#####################################################################
# Application Name, Binary Name, Application Directory            	#
#####################################################################
DEVICE=$(MCU)
APP_DIR = ./
USERAPP_NAME = BMF055FlightController_MW21

####################################################################
# Makefile variable Definitions                                    #
####################################################################
# Checking System dependencies and build context 
ifeq ($(ComSpec),)
  ifeq ($(COMSPEC),)
    $(info Platform: Linux)    
    syspath = $(1)
    GCC_ARM = $(TOOLDIR)/gnu_gcc_arm_linux
    RM      = rm -rf
    pquote  = $(addprefix $(QUOTE),$(addsuffix $(QUOTE),$(1)))
    settev  = export $(1)=$(2)    
    CP      = cp
    CPFLAGS = -R
  endif
else
endif
  $(info Platform: Windows)
  GCC_ARM = $(TOOLDIR)/GNU Tools ARM Embedded/5.3 2016q1
  RM      = "$(GCC_ARM)/bin/rm" -rf
  syspath = $(subst /,\,$(1))
  pquote  = $(addprefix $(QUOTE),$(addsuffix $(QUOTE),$(1)))
  settev  = set $(1)=$(2)
  CP      = xcopy
  CPFLAGS = /S /Y


####################################################################
# Build tools related variable definitions             
####################################################################
CC      = "$(GCC_ARM)/bin/arm-none-eabi-gcc"
LD      = "$(GCC_ARM)/bin/arm-none-eabi-ld"
AR      = "$(GCC_ARM)/bin/arm-none-eabi-ar"
OBJCOPY = "$(GCC_ARM)/bin/arm-none-eabi-objcopy"
CPP     = "$(GCC_ARM)/bin/arm-none-eabi-cpp" -E
CXX     = "$(GCC_ARM)/bin/arm-none-eabi-g++"
DUMP    = "$(GCC_ARM)/bin/arm-none-eabi-objdump" --disassemble
FLASH 	= "$(EACOMMANDERDIR)/eACommander"
SIZE    = "$(GCC_ARM)/bin/arm-none-eabi-size"
DOXY    = "$(DOXYGEN_DIR)/doxygen"
GRAPHVIZ= "$(GRAPHVIZ_DIR)/bin"
SPLINT  = $(TOOLDIR)/splint-win/bin/splint.exe
CCM     = $(TOOLDIR)/ccm/bin/CCM.exe
GREP    = grep
AWK     = awk
READELF   = "$(GCC_ARM)/bin/arm-none-eabi-readelf"

####################################################################
# Build directory related variable definitions             
####################################################################

OBJ_DIR = $(APP_DIR)_build
EXE_DIR = $(APP_DIR)_exe
LST_DIR = $(APP_DIR)_lst

EXE_FILE = $(EXE_DIR)/$(USERAPP_NAME).out
MAP_FILE = $(LST_DIR)/$(USERAPP_NAME).map
BIN_FILE = $(EXE_DIR)/$(USERAPP_NAME).bin

GCCVERSION = $(shell $(CC) -dumpversion)

####################################################################
# Flags                                                            #
####################################################################
# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.
DEPFLAGS = -MMD -MD -MP -MF $(@:.o=.d) 

# Add -Wa,-ahlms=$(LST_DIR)/$(@F:.o=.lst) to CFLAGS to produce assembly list files
CFLAGS += -D$(MCU) -DUSE_RTOS=$(USE_RTOS) -DMCU_FAMILY=$(MCU_FAMILY) -DCPU_CORE=$(CPU_CORE)
CFLAGS += -std=c99  -x c -Wall -Wextra -Wshadow -ffunction-sections -nostartfiles -mcpu=cortex-m0plus  -mthumb
CFLAGS += $(DEPFLAGS) -DBOARD=USER_BOARD -D__SAMD20J18__
# Use pipes instead of temporary files for communication between processes
CFLAGS    += -pipe
CFLAGS    += --param max-inline-insns-single=500
# Compile C files using the GNU99 standard.
CFLAGS    += -std=gnu99
# Don't use strict aliasing (very common in embedded applications).
CFLAGS    += -fno-strict-aliasing
# Separate each function and data into its own separate section to allow
# garbage collection of unused sections.       
CFLAGS    += -ffunction-sections -fdata-sections
# Always enable warnings. And be very careful about implicit
# declarations.
CFLAGS    += -Wall -Wstrict-prototypes -Wmissing-prototypes
CFLAGS    += -Wpointer-arith
# Various cflags.
CFLAGS    += -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS    += -Wmain -Wparentheses 
CFLAGS    += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS    += -Wuninitialized -Wunknown-pragmas -Wfloat-equal
# -Wundef
CFLAGS    += -Wshadow -Wbad-function-cast -Wwrite-strings
CFLAGS    += -Wsign-compare -Waggregate-return
CFLAGS    += -Wmissing-declarations
CFLAGS    += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS    += -Wpacked -Wredundant-decls -Wnested-externs -Wlong-long
CFLAGS    += -Wunreachable-code
CFLAGS    += -Wcast-align 

ASMFLAGS += -D$(DEVICE) -mcpu=cortex-m0plus -mthumb -nostartfiles -D__ASSEMBLY__ -nostartfiles -specs=nano.specs -D__SAMD20J18__
ASMFLAGS += -Ttext 0x0

LDFLAGS +=-Xlinker -Map=$(MAP_FILE) -Wl,--entry=Reset_Handler -D$(DEVICE) -mcpu=cortex-m0plus -lc -D__SAMD20J18__ --specs=rdimon.specs -specs=nano.specs -Wl,--gc-sections
LDFLAGS += -O0 -pipe -Wl,--gc-sections -Wl,-T ./samd20j18_flash.ld -mthumb -nostartfiles


#Standard C libraries. The order is important, don't change this. 
STDLIBS += -Wl,--start-group -lgcc -lc -lm -lnosys -Wl,--end-group

####################################################################
# Include paths                                                    #
####################################################################
INCLUDEPATHS += \
$(APP_INCLUDEPATHS) \
$(ASF_INCLUDEPATHS) \

####################################################################
# Source Files                                                     #
####################################################################
APP_SRC += \
src/config/startup_samd20.c \
$(ASF_SRC) \
$(APP_APP_SRC)

# put everything together
C_SRC  = $(APP_SRC)
C_FILES += $(notdir $(C_SRC))

# make list of source paths, sort also removes duplicates
C_PATHS = $(sort $(dir $(C_SRC)))
C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.o))
C_DEPS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.d))
#CLIB_DEPS = $(addprefix $(OBJ_DIR)/, $(CLIB_FILES:.c=.d))

vpath %.c $(C_PATHS)
vpath %.s $(S_PATHS)

####################################################################
# Make Targets                                                     #
####################################################################
# Default build is debug build
all:	debug

###############   DEBUG BUILD 		################################
debug:  CFLAGS += -DEBUG -O0 -g3 -gdwarf-2

debug::	BUILD_TYPE=Debug
debug::
	@echo "<=========== This is $(BUILD_TYPE) build for $(USERAPP_NAME) ====>"
debug:: $(OBJ_DIR) $(LST_DIR) $(EXE_DIR) $(BIN_FILE)

############### RELEASE BUILD - Optimize for space ##################
release:  CFLAGS += -DNDEBUG -Os
release:: BUILD_TYPE=Release
release::
	@echo "<=========== This is $(BUILD_TYPE) build for" $(PLATFORM) "Platform ====>"
	$(MAKE) -C $(SWPF_PATH) release 
	
release::  $(OBJ_DIR) $(LST_DIR) $(EXE_DIR) $(BIN_FILE)

####################################################################
# Common part of the Debug or Release make target                  #
####################################################################
# Create directories
$(OBJ_DIR):    
	@echo [ MKDIR ] $@
	@mkdir $(call syspath,$@)

$(EXE_DIR):
	@echo [ MKDIR ] $@
	@mkdir $(call syspath,$@)

$(LST_DIR):
	@echo [ MKDIR ] $@
	@mkdir $(call syspath,$@)

# Create objects from C SRC files
$(OBJ_DIR)/%.o: %.c
	@echo [ CC ] $<
	@$(CC) $(CFLAGS) $(addprefix -I,$(INCLUDEPATHS)) -c -o $@ $<

# Link
$(EXE_FILE): $(C_OBJS) 
	@echo [ LD ] $@
	@$(CC) $(LDFLAGS) $(C_OBJS) $(STDLIBS) -o $@
$(EXE_FILE_BS): $(C_OBJS)
	@echo [ LD ] $@
	@$(CC) $(LDFLAGS_BS) $(C_OBJS)  $(STDLIBS) -o $@

# Create binary file
$(BIN_FILE): $(EXE_FILE)
	@echo [ BIN ] $@
	@$(OBJCOPY) -O binary $< $@

# Create binary file which is bootloader supported	
$(BIN_FILE_BS): $(EXE_FILE_BS)
	@echo [ BIN ] $@
	@$(OBJCOPY) -O binary $< $@
	
############### CLEAN ##############################################	
clean:
	@echo [ RM ] $(OBJ_DIR) $(LST_DIR) $(EXE_DIR) 
	@$(RM) $(OBJ_DIR) $(LST_DIR) $(EXE_DIR) 

############### FLASH and RESET ################################
# Using these four commands, we can now flash the MCU using the command line and from Eclipse;
# Flash binary using eACommander and reset.
# The following target tested to be working with JLINKLite HW only.
# There is a separate reset make-target below if only reset is required.

flash:
	@echo [ FLASH_$(BIN_FILE) ]
	@echo h > jlink.cmd
	@echo si 1 > jlink.cmd	
	@echo speed 5000 >> jlink.cmd
	@echo sleep 100 >> jlink.cmd
	@echo r >> jlink.cmd
	@echo loadbin "$(BIN_FILE)", 0x00000000 >> jlink.cmd
	@echo verifybin "$(BIN_FILE)", 0x00000000 >> jlink.cmd
	@echo r >> jlink.cmd
	@echo sleep 100 >> jlink.cmd
	@echo g >> jlink.cmd
	@echo q >> jlink.cmd	
	-@  $(TOOLDIR)/Jlink/Jlink.exe -device $(MCU) jlink.cmd
	@$(RM) jlink.cmd
		
# reset the Microcontroller
reset:
	@$(FLASH) $< -r
	
# include auto-generated dependency files (explicit rules)
ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(C_DEPS) $(CLIB_DEPS)
endif
