TARGET_NAME ?= pipwm
TARGET_SHARED ?= $(TARGET_NAME)_shared
TARGET_STATIC ?= $(TARGET_NAME)_static

BUILD_DIR ?= build
SRC_BASE ?= ./
INC_BASE ?= ./

MKDIR_P ?= mkdir -p

SRCS := $(shell find $(SRC_BASE) -name "*.cpp" -or -name "*.c")
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(INC_BASE) -type d) $(shell find ../include/ -type d)
INC_FLAGS := $(addprefix -I ,$(INC_DIRS))

CPPFLAGS ?= $(INC_FLAGS) -MMD
CFLAGS ?= -Wall

all: static shared

static: $(BUILD_DIR)/$(TARGET_STATIC) 
shared: $(BUILD_DIR)/$(TARGET_SHARED)

$(BUILD_DIR)/$(TARGET_STATIC) : $(OBJS) ../build/libpipwm.a 
	$(info Making static linked executable: $(TARGET_STATIC))
	$(CC) -L /opt/vc/lib -lbcm_host -lm $^ -o $@ 

$(BUILD_DIR)/$(TARGET_SHARED): $(OBJS)
	$(info Making shared linked executable: $(TARGET_SHARED))
	$(CC) -L ../build/ -lpipwm $^ -o $@ 

$(BUILD_DIR)/%.c.o: %.c
	@$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@	

.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)