TARGET_EXEC ?= pipwm

BUILD_DIR ?= build
SRC_DIRS ?= ./

MKDIR_P ?= mkdir -p

SRCS := $(shell find $(SRC_DIRS) -name "*.cpp" -or -name "*.c")
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d) $(shell find ../include/ -type d)
INC_FLAGS := $(addprefix -I ,$(INC_DIRS))

LDFLAGS := -L ../build/ -lpipwm 
CPPFLAGS ?= $(INC_FLAGS) -MMD
CFLAGS ?= -Wall

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@ 

$(BUILD_DIR)/%.c.o: %.c
	@$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@	

.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)