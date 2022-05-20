GTKWAVE=/Applications/gtkwave.app/Contents/Resources/bin/gtkwave

all: task1 task2 task3

message_task1:
	@echo "--- TASK 1: SINGLE CYCLE PROCESSOR ---"

build_task1:
	iverilog -g2012 -o task1 riscvsingle.sv

run_task1:
	vvp task1

waves_task1:
	$(GTKWAVE) riscvsingle.vcd

clean_task1:
	rm riscvsingle.vcd task1

task1: message_task1 build_task1 run_task1 waves_task1 clean_task1


message_task2:
	@echo "--- TASK 2: MULTI CYCLE PROCESSOR CONTROLLER ---"

build_task2:
	iverilog -g2012 -o task2 controller_testbench.sv

run_task2:
	vvp task2

waves_task2:
	$(GTKWAVE) riscvmulticontroller.vcd

clean_task2:
	rm riscvmulticontroller.vcd task2

task2: message_task2 build_task2 run_task2 waves_task2 clean_task2


message_task3:
	@echo "--- TASK 3: MULTI CYCLE PROCESSOR ---"

build_task3:
	iverilog -g2012 -o task3 riscvmulti.sv riscv_testbench.sv

run_task3:
	vvp task3

waves_task3:
	$(GTKWAVE) riscvmulti.vcd

clean_task3:
	rm riscvmulti.vcd task3

task3: message_task3 build_task3 run_task3 waves_task3 clean_task3
