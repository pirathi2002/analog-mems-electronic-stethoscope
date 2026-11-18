# Build / flash helpers for the stethoscope firmware (uses arduino-cli).
#
#   make build             compile
#   make upload            compile + flash (PORT=/dev/ttyACM0 by default)
#   make upload BOARD=devkit   same, for an ESP32-S3 DevKitC (N16R8) test board
#   make upload BOARD=auto     detect port, chip, flash and PSRAM of any ESP32-S3 board
#   make detect            show what is connected (port, chip, flash, PSRAM)
#   make monitor           serial monitor (type "help")
#   make test              host unit tests of the DSP (g++ + Python)
#   make filters           regenerate FIR header + filter design plots
#   make viewer            real-time PC viewer   (make demo = no hardware)
#   make setup-tools       create the Python venv for firmware/tools
#   make vscode            fix VS Code IntelliSense (red include errors) for this machine

ARDUINO_CLI ?= $(shell command -v arduino-cli 2>/dev/null || echo $(HOME)/.local/bin/arduino-cli)
SKETCH      := stethoscope
# Serial port: first ESP32-looking port found (native USB first), override with PORT=...
PORT        ?= $(firstword $(wildcard /dev/ttyACM*) $(wildcard /dev/ttyUSB*) /dev/ttyACM0)
# BOARD=pcb    : our main board, IC1 = ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB octal PSRAM),
#                PC link over the native USB port (USB-Serial/JTAG)
# BOARD=pcb-uart : same main board, but programmed and linked through the J1 UART header
#                (TX/RX) with a 3.3 V USB-UART adapter; download mode by hand (P1 + SW1)
# BOARD=devkit : ESP32-S3-DevKitC-1 N16R8 for testing (16 MB flash, 8 MB octal PSRAM),
#                PC link over the "COM" (CH343/CP210x -> UART0) port at 921600 baud
# BOARD=auto   : ask the connected chip (tools/detect_board.py) and pick the settings
BOARD       ?= pcb
PY          := tools/.venv/bin/python
ifeq ($(BOARD),auto)
DETECTED    := $(shell $(PY) tools/detect_board.py --make $(if $(filter command line environment,$(origin PORT)),--port $(PORT)) || echo FAILED)
ifneq ($(words $(DETECTED)),4)
$(error board detection failed - see the message above, or use BOARD=pcb / BOARD=devkit)
endif
PORT        := $(word 1,$(DETECTED))
FQBN        := $(word 2,$(DETECTED))
EXTRA_FLAGS := $(word 3,$(DETECTED))
BUILD_DIR   := build/$(word 4,$(DETECTED))
$(info detected $(word 4,$(DETECTED)) on $(PORT))
else ifeq ($(BOARD),pcb-uart)
UPLOAD_SPEED ?= 460800
FQBN        ?= esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CPUFreq=240,UploadSpeed=$(UPLOAD_SPEED)
EXTRA_FLAGS := -DLINK_USE_UART=1
ifeq ($(origin PORT),file)
PORT        := $(firstword $(wildcard /dev/ttyUSB*) $(wildcard /dev/ttyACM*) /dev/ttyUSB0)
endif
else ifeq ($(BOARD),devkit)
FQBN        ?= esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CPUFreq=240
EXTRA_FLAGS := -DLINK_USE_UART=1
else
FQBN        ?= esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CPUFreq=240
EXTRA_FLAGS :=
endif
BUILD_DIR   ?= build/$(BOARD)

.PHONY: build upload detect monitor test filters viewer demo setup-tools vscode clean

build:
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --build-path $(BUILD_DIR) --warnings default \
		--build-property "compiler.cpp.extra_flags=$(EXTRA_FLAGS)" $(SKETCH)

upload: build
ifeq ($(BOARD),pcb-uart)
	$(PY) tools/uart_flash.py --port $(PORT) --fqbn "$(FQBN)" --build-dir $(BUILD_DIR)
else
	$(ARDUINO_CLI) upload --fqbn $(FQBN) --input-dir $(BUILD_DIR) -p $(PORT) $(SKETCH)
endif

detect: setup-tools
	$(PY) tools/detect_board.py $(if $(filter command line environment,$(origin PORT)),--port $(PORT))

# 921600 suits both links (native USB ignores the baud rate, UART0 runs at 921600)
monitor:
	$(ARDUINO_CLI) monitor -p $(PORT) -c baudrate=921600

test:
	python3 test/test_dsp.py

filters: setup-tools
	cd tools && .venv/bin/python design_filters.py --out filter_design

viewer: setup-tools
	cd tools && .venv/bin/python viewer.py

demo: setup-tools
	cd tools && .venv/bin/python viewer.py --demo

setup-tools: tools/.venv/bin/python

tools/.venv/bin/python:
	python3 -m venv tools/.venv
	tools/.venv/bin/pip install -r tools/requirements.txt

vscode:
	@test -f build/pcb/compile_commands.json || $(MAKE) build BOARD=pcb
	python3 tools/gen_vscode_config.py

clean:
	rm -rf $(BUILD_DIR)
