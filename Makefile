# SPDX-License-Identifier: MIT

SOURCES = src/pca9535.chip.c 
INCLUDES = -I . -I include
CHIP_JSON = src/pca9535io.chip.json

TARBALL  = dist/chip.tar.gz
TARGET  = dist/chip.out

.PHONY: all
all: clean $(TARBALL)

.PHONY: clean
clean:
		rm -rf dist

dist:
		mkdir -p dist

$(TARBALL): $(TARGET) dist/chip.json
	ls -l dist
	tar czf $(TARBALL) $(TARGET) dist/chip.json

dist/chip.json:
	cp $(CHIP_JSON) dist/chip.json

$(TARGET): dist $(SOURCES)
	  clang -Werror  $(INCLUDES) -o $(TARGET) $(SOURCES)
