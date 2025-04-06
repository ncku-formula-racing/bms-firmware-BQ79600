BUILD_DIR = build
TARGET := stm32f103c8
RTT_ADDR := 0x20000000
BUILD_TYPE := Debug

all:
	mkdir -p $(BUILD_DIR)
	- ./run_clang-format.sh 2> /dev/null &
	cd $(BUILD_DIR) && cmake .. -GNinja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	cd $(BUILD_DIR) && ninja
	@-mv $(BUILD_DIR)/compile_commands.json .

clean:
	rm -rf $(BUILD_DIR)

flash:
	@-pyocd flash -t $(TARGET) $(BUILD_DIR)/bms-master.elf

monitor:
	@-pyocd rtt -t $(TARGET) -a $(RTT_ADDR)

debug:
	@-pyocd gdbserver -t $(TARGET) --persist

.PHONY: all clean flash monitor
