ARM_TOOLS 	:= /home/maxim/arm_tools/gcc-arm-none-eabi-10-2020-q4-major/
ARM_PREFIX	:= arm-none-eabi
ARM_PATH	:= $(ARM_TOOLS)/bin/$(ARM_PREFIX)-

export CC	:= $(ARM_PATH)gcc
export LD	:= $(ARM_PATH)ld
export ASM	:= $(ARM_PATH)as
export MK	:= mkdir -p
export RM	:= rm -rf

export DEVICE	:= STM32F10X_MD
export BOARD	:= USE_RAW

ifeq ($(V), 1)
export Q 	:= 
else
export Q 	:= @
endif

ROOT_DIR	:= $(shell pwd)
BUILD_DIR	:= ./build
PERIPH_LIB_DIR	:= $(ROOT_DIR)/Libraries/STM32F10x_StdPeriph_Driver
USB_LIB_DIR	:= $(ROOT_DIR)/Libraries/STM32_USB-FS-Device_Driver
CMSIS_DIR	:= $(ROOT_DIR)/Libraries/CMSIS/
CMSIS_DEV_DIR	:= $(CMSIS_DIR)/Device/ST/STM32F10x
INC_DIRS	:=	$(ROOT_DIR)/inc				\
			$(PERIPH_LIB_DIR)/inc			\
			$(USB_LIB_DIR)/inc			\
			$(CMSIS_DIR)/Include			\
			$(CMSIS_DEV_DIR)/Include
CSRC_DIR	:= ./src

export INCLUDE	:= $(addprefix -I, $(INC_DIRS))

BOOT_SRC	:= ./device/startup_stm32f10x_md.s
BOOT_OBJ	:= $(BUILD_DIR)/$(notdir $(BOOT_SRC:%.s=%.o))

CSRC		:= $(wildcard $(CSRC_DIR)/*.c)
OBJS		:= $(addprefix $(BUILD_DIR)/, $(notdir $(CSRC:%.c=%.o)))

DEFS		:=	$(BOARD)				\
			$(DEVICE)				\
			USE_STDPERIPH_DRIVER

LD_SCRIPT	:= $(ROOT_DIR)/device/STM32F103C8_FLASH.ld
BIN_NAME	:= usb.elf

ifeq ($(DEVICE), STM32F10X_MD)
	export C_FLAGS	:= -mcpu=cortex-m3 -mthumb
	LD_LIB_PATHS	:=	$(ARM_TOOLS)/$(ARM_PREFIX)/lib/thumb/v7-m/nofp/		\
				$(ARM_TOOLS)/lib/gcc/arm-none-eabi/10.2.1/thumb/v7-m/nofp/
	LD_LIBS_INCL	:=	c m
else
	$(error Unknown device!)
endif

ifdef DEBUG
C_FLAGS += -O0 -g
endif

LD_LIB_DIRS		:= $(addprefix -L, $(LD_LIB_PATHS))
LD_LIBS			:= $(addprefix -l, $(LD_LIBS_INCL))

export DEFINES	:= $(addprefix -D, $(DEFS))


.PHONY: all periph_library usb_library clean

all: periph_library usb_library $(BOOT_OBJ) $(OBJS)
	$(Q)$(LD) $(OBJS) $(BOOT_OBJ) $(PERIPH_LIB_DIR)/build/* $(USB_LIB_DIR)/build/* $(LD_LIB_DIRS) $(LD_LIBS) -o $(BIN_NAME) -T$(LD_SCRIPT) -M > usb.map

$(BUILD_DIR)/%.o: $(CSRC_DIR)/%.c | $(BUILD_DIR)
	$(Q)$(CC) $(INCLUDE) $(DEFINES) $(C_FLAGS) -c $< -o $@

$(BOOT_OBJ): $(BOOT_SRC) | $(BUILD_DIR)
	$(Q)$(ASM) -c $< -o $@

periph_library:
	$(Q)$(MAKE) -C $(PERIPH_LIB_DIR) all

usb_library:
	$(Q)$(MAKE) -C $(USB_LIB_DIR) all

$(BUILD_DIR) ::
	$(Q)$(MK) $(BUILD_DIR)

flash_ocd:
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
		-c "program $(BIN_NAME) verify reset exit"

clean:
	$(Q)$(RM) $(BUILD_DIR)
	$(Q)$(RM) $(BIN_NAME)
	$(Q)$(MAKE) -C $(PERIPH_LIB_DIR) clean
	$(Q)$(MAKE) -C $(USB_LIB_DIR) clean
