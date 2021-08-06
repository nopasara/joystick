BUILD_DIR	:= ./build
SRC_DIR		:= ./src

SRC		:= $(wildcard $(SRC_DIR)/*.c)
OBJECTS 	:= $(addprefix $(BUILD_DIR)/,$(notdir $(SRC:%.c=%.o)))

.PHONY: all clean

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(Q)$(CC) $(INCLUDE) $(DEFINES) $(C_FLAGS) -c $< -o $@ 

all:	$(OBJECTS)

$(BUILD_DIR) ::
	$(Q)$(MK) $(BUILD_DIR)

clean:
	$(Q)$(RM) $(BUILD_DIR)
