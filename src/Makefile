# Extract project details from Git
#
# Project name from remote origin URL (fallback to 'unknown' if not available)
PRJ := $(shell git config --local remote.origin.url 2>/dev/null | sed -n 's#.*/\([^.]*\)\.git#\1#p' || echo "unknown")
# Current Git branch (sanitize non-alphanumeric characters to '-')
BRH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null | sed -E 's/[^a-zA-Z0-9]/-/g' || echo "unknown")
# Latest Git tag (version) without leading 'v'
VER := $(shell git describe --tags --match "v[0-9]*" --match "[0-9]*" --abbrev=0 2>/dev/null | sed -E 's/^v?([0-9]+\.[0-9]+\.[0-9]+)/\1/' || echo "0.0.0")
# Append branch name if not on 'main' or 'master'
PRE := $(shell if [ "$(BRH)" != "main" ] && [ "$(BRH)" != "master" ]; then echo "-$(BRH)"; fi)
# Append branch name to version only if not already suffixed
VER := $(VER)$(PRE)
# Append commit hash if current commit is not an exact tag
COMMIT := $(shell git rev-parse --short HEAD 2>/dev/null || echo "unknown")
VER := $(shell if ! git describe --tags --exact-match >/dev/null 2>&1; then echo "$(VER)+$(COMMIT)"; else echo "$(VER)"; fi)
# Lower-case based executable name
EXE_NAME := $(shell echo $(PRJ) | tr '[:upper:]' '[:lower:]')

# Output Items
OUT := $(EXE_NAME)						# Normal release binary
TEST_OUT :=	$(EXE_NAME)_test			# Debug/test binary
INSTALL_PATH := /usr/local				# Installation directory for the binary
INI_PATH :=	/usr/local/etc/wsprrypi.ini	# INI file for test target
CALL_SIGN := AA0NT						# Callsign
GRID_SQUARE := EM18						# Maidenhead Grid Square
TX_POWER := 20							# Transmit Power in dBm
TEST_FREQ := 7040100					# Frequency for test tone
# Strip whitespace from comments
OUT := $(strip $(OUT))
TEST_OUT := $(strip $(TEST_OUT))
INSTALL_PATH := $(strip $(INSTALL_PATH))
INI_PATH := $(strip $(INI_PATH))
CALL_SIGN := $(strip $(CALL_SIGN))
TX_POWER := $(strip $(TX_POWER))
GRID_SQUARE := $(strip $(GRID_SQUARE))
TEST_FREQ := $(strip $(TEST_FREQ))

# Output directories
OBJ_DIR_RELEASE = build/obj/release
OBJ_DIR_DEBUG   = build/obj/debug
DEP_DIR         = build/dep
BIN_DIR		 	= build/bin

# Collect source files
CPP_SOURCES := $(shell find . -name "*.cpp" ! -path "./*/main.cpp")
# Collect object files
CPP_OBJECTS := $(patsubst %.cpp,$(OBJ_DIR_RELEASE)/%.o,$(CPP_SOURCES))
# Linker Flags
LDFLAGS := -lpthread -latomic
# Collect dependency files
DEPFILES := $(wildcard $(DEP_DIR)/**/*.d)
# Include dependencies if they exist
-include $(DEPFILES)

# Compiler Executables
CXX = g++

# Allow threading using all available processors
MAKEFLAGS := -j$(nproc)

# Compiler Flags
#
# Common flags for both C and C++ compilers
COMMON_FLAGS := -Wall -Werror -fmax-errors=10 -MMD -MP

# Include paths for libraries
# CXXFLAGS += -I$(abspath ./INI-Handler/src)
# CXXFLAGS += -I$(abspath ./LCBLog/src)
# CXXFLAGS += -I$(abspath ./MonitorFile/src)
# CXXFLAGS += -I$(abspath ./WSPR-Message/src)
# CXXFLAGS += -I$(abspath ./Singleton/src)
# CXXFLAGS += -I$(abspath ./Broadcom-Mailbox/src)

# Target flags
CXX_RELEASE_FLAGS := $(COMMON_FLAGS) $(CXXFLAGS) -O2			# Release optimized
CXX_DEBUG_FLAGS := $(COMMON_FLAGS) $(CXXFLAGS) -g -DDEBUG_BUILD	# Debug flags

# Strip whitespaces
CXX_DEBUG_FLAGS := $(strip $(CXX_DEBUG_FLAGS))
CXX_RELEASE_FLAGS := $(strip $(CXX_RELEASE_FLAGS))

# Enable verbose output if VERBOSE=1 is specified during the build
VERBOSE ?= 0
ifeq ($(VERBOSE), 1)
	Q :=
else
	Q := @
endif

##
# Compile and Link
##

# Compile C++ source files for debug
$(OBJ_DIR_DEBUG)/%.o: %.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $<)
	$(Q)echo "Compiling (debug) $< into $@"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

# Link the debug binary
build/bin/$(TEST_OUT): $(patsubst %.cpp,$(OBJ_DIR_DEBUG)/%.o,$(CPP_SOURCES))
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: $(TEST_OUT)"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o build/bin/$(TEST_OUT) $(LDFLAGS)

# Compile C++ source files (release)
$(OBJ_DIR_RELEASE)/%.o: %.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $<)
	$(Q)echo "Compiling (release) $< into $@"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

# Link the final binary (release)
build/bin/$(OUT): $(CPP_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking release binary: $(OUT)"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) $^ -o $@ $(LDFLAGS)

##
# Make Targets
##

# Force the fefault to be release (order matters in targets otherwise)
.DEFAULT_GOAL := release

# Clean dirs
.PHONY: clean
clean:
	$(Q)echo "Cleaning up build artifacts."
	$(Q)rm -rf build

# Release target
.PHONY: release
release: build/bin/$(OUT)
	$(Q)echo "Release build completed successfully."

# Debug target
.PHONY: debug
debug: build/bin/$(TEST_OUT)
	$(Q)echo "Debug build completed successfully."

# Test with INI target
.PHONY: test
test: debug
	$(Q)echo "Running PPM demo."
	$(Q)./build/bin/$(TEST_OUT)

# Show only user-defined macros
.PHONY: macros
macros:
	$(Q)echo "Defined Project Macros:"
	$(Q)$(CXX) $(CXXFLAGS) -dM -E -x c++ /dev/null | grep -v '^#define _' || true

# Lint target
.PHONY: lint
lint:
	$(Q)echo "Running static analysis with cppcheck."
	$(Q)cppcheck --platform=unix32 --std=c++17 --enable=all --inconclusive --force --inline-suppr --quiet $(CPP_SOURCES)

# Display available make targets
.PHONY: help
help:
	$(Q)echo "Available targets:"
	$(Q)echo "  all        Build the project (default: release)."
	$(Q)echo "  clean      Remove build artifacts."
	$(Q)echo "  debug      Build with debugging symbols."
	$(Q)echo "  test       Run a test monitoring PPM."
	$(Q)echo "  lint       Run static analysis."
	$(Q)echo "  macros     Show defined project macros."
	$(Q)echo "  release    Build optimized for production."
	$(Q)echo "  help       Show this help message."
