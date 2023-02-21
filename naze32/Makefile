###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the baseflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
# 

###############################################################################
# Things that the user might override on the commandline
#

# The target to build, must be one of NAZE or CJMCU
TARGET		?= NAZE

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

###############################################################################
# Things that need to be maintained as the source changes
#

VALID_TARGETS = NAZE CJMCU

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)/src
CMSIS_DIR	 = $(ROOT)/lib/CMSIS
STDPERIPH_DIR	 = $(ROOT)/lib/STM32F10x_StdPeriph_Driver
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj

# Source files common to all targets
COMMON_SRC	 = buzzer.c \
		   cli.c \
		   config.c \
		   imu.c \
		   main.c \
		   mixer.c \
		   mw.c \
		   link_driver.c \
		   sensors.c \
		   serial.c \
		   rxmsp.c \
		   drv_gpio.c \
		   drv_i2c.c \
		   drv_i2c_soft.c \
		   drv_system.c \
		   drv_serial.c \
		   drv_spektrum.c \
		   drv_uart.c \
		   printf.c \
		   utils.c \
		   fw_nav.c \
		   ibus.c \
		   sbus.c \
		   sumd.c \
		   spektrum.c \
		   pif.c \
		   pif_comm.c \
		   pif_i2c.c \
		   pif_list.c \
		   pif_log.c \
		   pif_ring_buffer.c \
		   pif_task.c \
		   pif_timer.c \
		   pif_noise_filter.c \
		   pif_noise_filter_int32.c \
		   pif_gps.c \
		   pif_gps_ublox.c \
		   pif_msp.c \
		   pif_rc.c \
		   pif_rc_ibus.c \
		   pif_rc_ppm.c \
		   pif_rc_pwm.c \
		   pif_rc_sbus.c \
		   pif_rc_spektrum.c \
		   pif_rc_sumd.c \
		   pif_bmp280.c \
		   pif_hc_sr04.c \
		   pif_hmc5883.c \
		   pif_imu_sensor.c \
		   pif_mpu30x0.c \
		   pif_mpu60x0.c \
		   pif_mpu6500.c \
		   pif_buzzer.c \
		   pif_storage.c \
		   pif_storage_var.c \
		   startup_stm32f10x_md_gcc.S \
		   $(CMSIS_SRC) \
		   $(STDPERIPH_SRC)

# Source files for full-featured systems
HIGHEND_SRC	 = gps.c \
		   drv_softserial.c \
		   telemetry_common.c \
		   telemetry_frsky.c \
		   telemetry_hott.c

# Source files for the NAZE target
NAZE_SRC	 = drv_adc.c \
		   drv_adxl345.c \
		   drv_ak8975.c \
		   drv_bma280.c \
		   drv_bmp280.c \
		   drv_hcsr04.c \
		   drv_hmc5883l.c \
		   drv_ledring.c \
		   drv_mma845x.c \
		   drv_mpu.c \
		   drv_l3g4200d.c \
		   drv_pwm.c \
		   drv_spi.c \
		   drv_timer.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

# Source files for the CJMCU target
CJMCU_SRC	 = drv_adc.c \
		   drv_mpu.c \
		   drv_hmc5883l.c \
		   drv_pwm.c \
		   drv_timer.c \
		   $(COMMON_SRC)

# In some cases, %.s regarded as intermediate file, which is actually not.
# This will prevent accidental deletion of startup code.
.PRECIOUS: %.s

# Search path for baseflight sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/baseflight_startups

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			               $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

VPATH		:= $(VPATH): \
			$(ROOT)/../fc/source: \
			$(ROOT)/../../pif/source/core: \
			$(ROOT)/../../pif/source/filter: \
			$(ROOT)/../../pif/source/gps: \
			$(ROOT)/../../pif/source/protocol: \
			$(ROOT)/../../pif/source/rc: \
			$(ROOT)/../../pif/source/sensor: \
			$(ROOT)/../../pif/source/sound: \
			$(ROOT)/../../pif/source/storage

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy

#
# Tool options.
#
INCLUDE_DIRS	 = $(SRC_DIR) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \
		   $(ROOT)/../fc/include \
		   $(ROOT)/../../pif/include

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -Og
LTO_FLAGS	 = -ggdb3 $(OPTIMIZE)
else
OPTIMIZE	 = -Os
LTO_FLAGS	 = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -std=gnu99 \
		   -Wall -Wextra -Wshadow -Wunsafe-loop-optimizations \
		   -ffunction-sections \
		   -fdata-sections \
		   -DSTM32F10X_MD \
		   -DUSE_STDPERIPH_DRIVER \
		   -D$(TARGET) \
		   -D__PIF_NO_LOG__ \
		   -D__PIF_LOG_COMMAND__ \
		   -DPIF_LOG_LINE_SIZE=128 \
		   -DPIF_LOG_TX_BUFFER_SIZE=128

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LD_SCRIPT	 = $(ROOT)/stm32_flash.ld
LDFLAGS		 = -lm \
		   -nostartfiles \
		   -lc \
		   -lnosys \
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif


TARGET_HEX	 = $(BIN_DIR)/baseflight_$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/baseflight_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP   = $(OBJECT_DIR)/baseflight_$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

MKDIR_OBJDIR = @mkdir -p $(dir $@)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 
$(OBJECT_DIR)/$(TARGET)/%.o): %.S
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

clean:
	rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)


unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

unbrick: unbrick_$(TARGET)

help:
	@echo ""
	@echo "Makefile for the baseflight firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
