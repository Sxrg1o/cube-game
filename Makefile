CC = gcc
RM = rm -rf

TARGET = cube
SRC_DIR = src
BUILD_DIR = build
INCLUDE_DIR = include

CFLAGS = -Wall -g -I$(INCLUDE_DIR) -c

LDFLAGS = -lraylib -lm

SRCS = $(wildcard $(SRC_DIR)/*.c)

OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))

all: $(BUILD_DIR) $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $< -o $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

clean:
	$(RM) $(BUILD_DIR)

.PHONY: all clean
