# Arduino Nano (ATmega328P) settings
MCU = atmega328p
F_CPU = 16000000UL
BAUD = 115200
PROGRAMMER = arduino
PORT = /dev/ttyUSB0

# Compiler settings
CC = avr-gcc
CXX = avr-g++
OBJCOPY = avr-objcopy
SIZE = avr-size
AVRDUDE = avrdude

CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra
CXXFLAGS = $(CFLAGS) -std=c++11 -fno-exceptions -fno-threadsafe-statics
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

# Arduino paths (update these to match your system)
ARDUINO_DIR = /usr/share/arduino/hardware/arduino/avr
# Or try: ~/.arduino15/packages/arduino/hardware/avr/1.8.6
# macOS: /Applications/Arduino.app/Contents/Java/hardware/arduino/avr

ARDUINO_CORE = $(ARDUINO_DIR)/cores/arduino
ARDUINO_VARIANT = $(ARDUINO_DIR)/variants/standard

# Include paths
INCLUDES = -I. -Ilibs -I$(ARDUINO_CORE) -I$(ARDUINO_VARIANT)

# Arduino core sources (compiled from system location)
ARDUINO_C_SOURCES = $(wildcard $(ARDUINO_CORE)/*.c)
ARDUINO_CXX_SOURCES = $(wildcard $(ARDUINO_CORE)/*.cpp)
ARDUINO_OBJECTS = $(patsubst $(ARDUINO_CORE)/%.c,build/arduino/%.o,$(ARDUINO_C_SOURCES)) \
                  $(patsubst $(ARDUINO_CORE)/%.cpp,build/arduino/%.o,$(ARDUINO_CXX_SOURCES))

# Your library sources
LIB_SOURCES = $(wildcard libs/*/*.cpp)
LIB_OBJECTS = $(patsubst libs/%.cpp,build/libs/%.o,$(LIB_SOURCES))

# Create build directories
$(shell mkdir -p build/arduino build/libs build/tests)

# Build Arduino core library
build/arduino_core.a: $(ARDUINO_OBJECTS)
	avr-ar rcs $@ $^

# Compile Arduino core from system location
build/arduino/%.o: $(ARDUINO_CORE)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/arduino/%.o: $(ARDUINO_CORE)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile your libraries
build/libs/%.o: libs/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile main and tests
build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

build/tests/%.o: tests/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Main target
main: build/main.o $(LIB_OBJECTS) build/arduino_core.a
	$(CXX) $(LDFLAGS) $^ -o build/main.elf
	$(OBJCOPY) -O ihex -R .eeprom build/main.elf build/main.hex
	$(SIZE) --mcu=$(MCU) -C build/main.elf

# Test targets
test_%: build/tests/test_%.o $(LIB_OBJECTS) build/arduino_core.a
	$(CXX) $(LDFLAGS) $^ -o build/$@.elf
	$(OBJCOPY) -O ihex -R .eeprom build/$@.elf build/$@.hex
	$(SIZE) --mcu=$(MCU) -C build/$@.elf

# Upload targets
upload-main: main
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:build/main.hex:i

upload-test_%: test_%
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:build/test_$*.hex:i

# Clean
clean:
	rm -rf build

.PHONY: clean upload-main list-tests