TARGET_EXEC ?= pipwm

BUILD_DIR ?= build
SRC_DIRS ?= source

SRCS := $(shell find $(SRC_DIRS) -name "*.cpp" -or -name "*.c")
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d) /opt/vc/include
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

LDFLAGS := -L/opt/vc/lib -lbcm_host 
CPPFLAGS ?= $(INC_FLAGS) -Wall

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@


.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p