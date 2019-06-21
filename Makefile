TARGET_NAME ?= pipwm
TARGET_SHARED ?= lib$(TARGET_NAME).so
TARGET_STATIC ?= lib$(TARGET_NAME).a

BUILD_DIR ?= build
SRC_DIRS ?= source

MKDIR_P ?= mkdir -p

SRCS := $(shell find $(SRC_DIRS) -name "*.cpp" -or -name "*.c")
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d) /opt/vc/include
INC_FLAGS := $(addprefix -I ,$(INC_DIRS))

LDFLAGS := -L /opt/vc/lib -lbcm_host -lm
CPPFLAGS ?= $(INC_FLAGS) -MMD
CFLAGS ?= -Wall

all: static shared

static: $(BUILD_DIR)/$(TARGET_STATIC) 
shared: $(BUILD_DIR)/$(TARGET_SHARED)

$(BUILD_DIR)/$(TARGET_STATIC): $(OBJS)
	$(AR) rcs $@ $^ 

$(BUILD_DIR)/lib$(TARGET_NAME).so: $(OBJS)
	$(CC) $(LDFLAGS) -shared -fPIC $^ -o $@ 

$(BUILD_DIR)/%.c.o: %.c
	@$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@	

.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)
