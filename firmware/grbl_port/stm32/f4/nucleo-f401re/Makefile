##
## This file is part of the grbl_port_opencm3 project.
##
## Copyright (C) 2017 Angelo Di Chello
##
## Grbl_port_opencm3 is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Grbl_port_opencm3 is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with Grbl_port_opencm3.  If not, see <http://www.gnu.org/licenses/>.
##


###############################################################################
# Source files

OBJ_DIR ?= ./build_dir/
SRC_DIR ?= ./
COMMON_SRC_DIR ?= ../../../common_src/
MKDIR_P = mkdir -p

SRCS := reset_vector.c flash.c   

COMMON_SRCS := coolant_control.c \
               nuts_bolts.c \
               report.c \
               stepper.c \
               settings.c \
               serial.c \
               system.c \
               print.c \
               limits.c \
               gcode.c \
               motion_control.c \
               protocol.c \
               planner.c \
               probe.c \
               test_nucleo.c \
               spindle_control.c 

LDSCRIPT = ./nucleo-f401re_ramcode.ld

COMMON_OBJS := $(COMMON_SRCS:%.c=$(OBJ_DIR)%.o)
OBJS := $(SRCS:%.c=$(OBJ_DIR)%.o)
BINARY = $(OBJ_DIR)main
OBJS += $(COMMON_OBJS)

###############################################################################
# C & C++ preprocessor common flags
CPPFLAGS  += -I. -I./cpu_map -I./defaults -I$(COMMON_SRC_DIR)

###############################################################################
# Used libraries
LDLIBS += -lm 

clean : clean_obj

include ../Makefile.include	

$(BINARY).elf: $(OBJS) $(LDSCRIPT) $(LIB_DIR)/lib$(LIBNAME).a
	@#printf "  LD      $(*).elf\n"
	$(Q)$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $(BINARY).elf

$(BINARY)%.map: $(OBJS) $(LDSCRIPT) $(LIB_DIR)/lib$(LIBNAME).a
	@#printf "  LD      $(*).elf\n"
	$(Q)$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $(BINARY).elf

$(OBJ_DIR):
	$(Q)$(MKDIR_P) $@

$(OBJ_DIR)%.o: %.c | $(OBJ_DIR)
	@#printf "  CC      $(*).c\n"
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(OBJ_DIR)$(*).o -c $(*).c
	
$(OBJ_DIR)%.o: $(COMMON_SRC_DIR)%.c | $(OBJ_DIR)
	@#printf "  CC      $(*).c\n"
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(OBJ_DIR)$(*).o -c $(COMMON_SRC_DIR)$(*).c
	
clean_obj:
	@#printf "  CLEAN\n"
	$(Q)$(RM) $(OBJ_DIR)*.o $(OBJ_DIR)*.d $(OBJ_DIR)*.elf \
			$(OBJ_DIR)*.bin $(OBJ_DIR)*.hex $(OBJ_DIR)*.list $(OBJ_DIR)*.map

.PHONY: clean_obj

