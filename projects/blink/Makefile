# Project config
SOURCE_FILES = main.c
# Do not make this the same name as the source file!
PROJECT_NAME = blink
# Clean will remove this directory, so make sure there aren't other files in it
BUILD_DIR   = bin

# EFM8 files
EFM8DIR = ../..
EFM8TOOLS = $(EFM8DIR)/Tools/Windows

# Programs and flags
RM          = rm
MKDIR       = mkdir
CC          = sdcc
CFLAGS      = -I$(EFM8DIR)/Include
TARGET_ARCH = -mmcs51
HEX2BOOT    = $(EFM8TOOLS)/hex2boot
EFM8LOAD    = $(EFM8TOOLS)/efm8load

# Files
OBJECTS    = $(BUILD_DIR)/$(SOURCE_FILES:.c=.rel)
BINARY     = $(BUILD_DIR)/$(PROJECT_NAME).ihx
BOOTRECORD = $(BUILD_DIR)/$(PROJECT_NAME).efm8

all: $(BINARY)

clean:
	@$(RM) -rf $(BUILD_DIR)

program: $(BOOTRECORD)
	$(EFM8LOAD) $(BOOTRECORD)

$(BUILD_DIR)/%.rel : %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	$(MKDIR) $@

$(BINARY): $(OBJECTS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

$(BOOTRECORD): $(BINARY)
	$(HEX2BOOT) $< -o $@ 

.PHONY: all clean program