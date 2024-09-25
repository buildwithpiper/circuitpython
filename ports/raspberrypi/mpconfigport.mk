# All raspberrypi ports have longints.
LONGINT_IMPL = MPZ

CIRCUITPY_OPTIMIZE_PROPERTY_FLASH_SIZE ?= 1
# CYW43 support does not provide settable MAC addresses for station or AP.
CIRCUITPY_WIFI_RADIO_SETTABLE_MAC_ADDRESS = 0

CIRCUITPY_ALARM ?= 1
CIRCUITPY_RP2PIO ?= 1
CIRCUITPY_NEOPIXEL_WRITE ?= $(CIRCUITPY_RP2PIO)
CIRCUITPY_FLOPPYIO ?= 1
CIRCUITPY_FRAMEBUFFERIO ?= $(CIRCUITPY_DISPLAYIO)
CIRCUITPY_FULL_BUILD ?= 1
CIRCUITPY_AUDIOMP3 ?= 1
CIRCUITPY_BITOPS ?= 1
CIRCUITPY_HASHLIB ?= 1
CIRCUITPY_HASHLIB_MBEDTLS ?= 1
CIRCUITPY_IMAGECAPTURE ?= 0
CIRCUITPY_MAX3421E ?= 0
CIRCUITPY_MEMORYMAP ?= 1
CIRCUITPY_PWMIO ?= 1
CIRCUITPY_RGBMATRIX ?= $(CIRCUITPY_DISPLAYIO)
CIRCUITPY_ROTARYIO ?= 1
CIRCUITPY_ROTARYIO_SOFTENCODER = 1
CIRCUITPY_SYNTHIO_MAX_CHANNELS = 12
CIRCUITPY_USB_HOST ?= 0
CIRCUITPY_USB_VIDEO ?= 0

# Things that need to be implemented.
CIRCUITPY_FREQUENCYIO = 0

# Use PWM internally
CIRCUITPY_I2CTARGET = 1
CIRCUITPY_NVM = 1

# Use PIO internally
CIRCUITPY_PULSEIO ?= 1
CIRCUITPY_WATCHDOG ?= 1

# Use of analogbufio
CIRCUITPY_ANALOGBUFIO = 1

# Audio via PWM
CIRCUITPY_AUDIOIO = 0
CIRCUITPY_AUDIOBUSIO ?= 1
CIRCUITPY_AUDIOCORE ?= 1
CIRCUITPY_AUDIOPWMIO ?= 1

CIRCUITPY_AUDIOMIXER ?= 1

INTERNAL_LIBM = 1

CIRCUITPY_BUILD_EXTENSIONS ?= uf2

# Number of USB endpoint pairs.
USB_NUM_ENDPOINT_PAIRS = 8

INTERNAL_FLASH_FILESYSTEM = 1
CIRCUITPY_SETTABLE_PROCESSOR_FREQUENCY = 1

# Usually lots of flash space available
CIRCUITPY_MESSAGE_COMPRESSION_LEVEL ?= 1

# (ssl is selectively enabled but it's always the mbedtls implementation)
CIRCUITPY_SSL_MBEDTLS = 1



# https://github.com/adafruit/Adafruit_CircuitPython_Register
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_Register

# https://github.com/adafruit/Adafruit_CircuitPython_HID
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_HID

# https://github.com/adafruit/Adafruit_CircuitPython_Motor
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_Motor

# https://github.com/adafruit/Adafruit_CircuitPython_TCS34725
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_TCS34725

# https://github.com/adafruit/Adafruit_CircuitPython_MCP9808
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_MCP9808

# https://github.com/adafruit/Adafruit_CircuitPython_MPU6050
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_MPU6050

# https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_NeoPixel

# https://github.com/buildwithpiper/circuitpython-range-finder-library.git
FROZEN_MPY_DIRS += $(TOP)/frozen/Piper_Distance_sensor

# https://github.com/buildwithpiper/circuitpython-motor-module-library.git
FROZEN_MPY_DIRS += $(TOP)/frozen/Piper_Motor_Module

# https://github.com/buildwithpiper/circuitpython-lightshow-library.git
FROZEN_MPY_DIRS += $(TOP)/frozen/Piper_LightShow

# https://github.com/buildwithpiper/circuitpython-heart-sensor-library.git
FROZEN_MPY_DIRS += $(TOP)/frozen/Piper_Heart_Sensor

# https://github.com/buildwithpiper/circuitpython-piper-make-library.git
FROZEN_MPY_DIRS += $(TOP)/frozen/Piper_Blockly_Library
