_makefile_path := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# TODO: make this configurable or at least hidden during delivery
export VERILATOR_ROOT ?= /home/htan31/tool/verilator/v5.028

OUTDIR ?= $(_makefile_path)/output
TARGET ?= simv

_cpp_top = $(_makefile_path)/verif/tb/testbench.cpp
_src = $(wildcard $(_makefile_path)/verif/tb/*.sv) \
	   $(wildcard $(_makefile_path)/src/*.sv)

$(OUTDIR)/$(TARGET): $(_cpp_top) $(_src)
	$(VERILATOR_ROOT)/bin/verilator --cc --exe --build -j 0 -Wall $(_cpp_top) $(_src) --timing -Mdir $(OUTDIR) -o $(TARGET) -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-SYNCASYNCNET

run: $(OUTDIR)/$(TARGET)
	@$(OUTDIR)/$(TARGET)

all: run