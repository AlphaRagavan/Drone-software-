# NIDAR Autonomous Search & Rescue Drone - High-Level Makefile
.PHONY: all native cross test clean deploy help

BUILD_DIR ?= build
CROSS_BUILD_DIR ?= build_arm64

all: native

help:
	@echo "NIDAR Autonomous Drone Build Targets:"
	@echo "  make native   - Compile natively for current architecture (Linux x86_64 or ARM)"
	@echo "  make cross    - Cross-compile for ARM64 using toolchain-aarch64.cmake"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make deploy   - Package and deploy to companion computer"

native:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Release
	@$(MAKE) -C $(BUILD_DIR) -j4

cross:
	@chmod +x cross_compile.sh
	@./cross_compile.sh

clean:
	@rm -rf $(BUILD_DIR) $(CROSS_BUILD_DIR) deploy
	@echo "Build directories cleaned."

deploy: cross
	@echo "Deploying to target drone on isolated 5GHz Wi-Fi link..."
	@scp deploy/nidar_onboard pi@nidar-drone.local:/home/pi/nidar_system/
