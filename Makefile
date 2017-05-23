PREFIX ?= $(shell pwd)/build
LIB_INST ?= /usr/lib
INC_INST ?= /usr/include
BIN_INST ?= /usr/bin
PWD := $(shell pwd)/

UTILS_DIR := Utils
OCS_DIRS := $(notdir $(shell find . -maxdepth 1 -type d -not -name '.*' -not -name $(UTILS_DIR)))
EXTRA_INCLUDES := $(UTILS_DIR) ocsgpio/libocsgpio

ifneq ($(PC_INCLUDE),)
EXTRA_INCLUDES += $(UTILS_DIR)/pc_include
endif

RPATH_ARG := -Wl,-rpath-link=
OCS_INC = $(addprefix -I$(PWD)/, $(OCS_DIRS) $(EXTRA_INCLUDES))
OCS_LIB = $(addsuffix /lib, $(addprefix -L$(PWD)/, $(OCS_DIRS)))
OCS_RPATH = $(addsuffix /lib, $(addprefix $(RPATH_ARG)$(PWD)/, $(OCS_DIRS)))

OCS_CFLAGS = $(OCS_INC) $(CFLAGS)
OCS_LDFLAGS = $(OCS_LIB) $(OCS_RPATH) $(LDFLAGS)

ifneq ($(PC_DEBUG),)
OCS_CFLAGS += -DPC_DEBUG
endif

MAKE_ARGS = 'CFLAGS=$(OCS_CFLAGS)' 'LDFLAGS=$(OCS_LDFLAGS)'
BUILD_CMD = $(MAKE) $(MAKE_ARGS)
INST_ARGS := PREFIX=$(PREFIX) LIB_INST=$(LIB_INST) INC_INST=$(INC_INST) BIN_INST=$(BIN_INST)

OCS_BUILD := \
	ocslock \
	ocslog \
	ocsfrui2c \
	ocs-fru

.PHONY: all
all: $(OCS_BUILD)

# Build rules for OCS libraries and applications.
.PHONY: ocslock
ocslock:
	$(BUILD_CMD) -C SemLock
	
.PHONY: ocslog
ocslog: ocslock
	$(BUILD_CMD) -C Ocslog
	
.PHONY: ocsfrui2c
ocsfrui2c: ocslog
	$(BUILD_CMD) -C frui2clib
	
.PHONY: ocs-fru
ocs-fru: ocslog ocsfrui2c
	$(BUILD_CMD) -C fru-util

.PHONY: clean
clean:
	rm -rf build
	for ocs in $(OCS_DIRS); do \
		$(MAKE) -C $$ocs clean; \
	done

.PHONY: install
install:
	install -d $(PREFIX)/$(LIB_INST)
	install -d $(PREFIX)/$(INC_INST)
	install -d $(PREFIX)/$(BIN_INST)
	for ocs in $(OCS_DIRS); do \
		$(MAKE) $(INST_ARGS) -C $$ocs install; \
	done
