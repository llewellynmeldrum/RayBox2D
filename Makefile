# Default goal builds the final program.
all: bin/RayBox2D

# Mark non-file targets as always-out-of-date.
.PHONY: all run clean

# Program name and host check.
PROG := RayBox2D
UNAME := $(shell uname)

# Fail fast on non-macOS since libs are macOS/arm64.
ifneq ($(UNAME),Darwin)
$(error Non-macOS detected. This build uses arm64 macOS static libs.)
endif

# Toolchain and language mode.
CC := clang
CSTD := c2x

# Source files and their matching object files.
SRC  := $(wildcard src/*.c)                           # e.g., src/main.c src/game.c
OBJ  := $(patsubst src/%.c,build/%.o,$(SRC))          # -> build/main.o build/game.o

# Include search paths and compile flags.
INCLUDES := -Iinclude -Ithird_party
CFLAGS   := -std=$(CSTD) $(INCLUDES) -O3

# Static libs and Apple frameworks for raylib on macOS.
RAYLIB := lib/raylib_arm64.a
BOX2D	:= lib/box2d_arm64.a
FRAMEWORKS := -framework CoreVideo -framework IOKit -framework Cocoa -framework OpenGL
# If you add Box2D (C++), use clang++ to link and add the static lib after objects.
# CXX := clang++
# BOX2D := lib/box2d_arm64.a

# Link rule: build the final binary from all objects.
# "bin" is an order-only prerequisite so the dir exists but doesn't force relink.
bin/$(PROG): $(OBJ) | bin
	$(CC) $(OBJ) $(RAYLIB) $(BOX2D) $(FRAMEWORKS) -o $@
	# With Box2D:
	# $(CXX) $(OBJ) $(RAYLIB) $(BOX2D) $(FRAMEWORKS) -o $@

# Compile rule: one C source -> one object.
# "build" is order-only to ensure the dir exists.
build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

# Directory creators (no-op if already present).
build:
	mkdir -p build

bin:
	mkdir -p bin

# Convenience target: ensure program exists, then run it.
run: bin/$(PROG)
	./$<

# Remove intermediates and the final binary.
clean:
	rm -rf build bin/$(PROG)
