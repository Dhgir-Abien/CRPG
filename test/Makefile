################################################################################
##
## Filename: Makefile
## Project:	 Constrained Random Pattern Generation
##
## Creator:	 Dhgir.Abine@gmail.com
##
################################################################################

VERILATOR := verilator
SRC_DIR := $(CRPG_PATH)src
VFLAGS += --cc --public --trace --assert -O3
VFLAGS += -CFLAGS -o3
VFLAGS += -CFLAGS -I$(CRPG_PATH)include
VFLAGS += -y build
VFLAGS += --Mdir build/obj_dir 
VFLAGS += 
VFLAGS += --exe $(SRC_DIR)/main.cpp
VERILATE=$(VERILATOR) $(VFLAGS)
SUBMAKE := $(MAKE) --no-print-directory --directory=build/obj_dir -f

.PHONY: all
all: clean_build init design_info design_pre sim_rtl gen_include sim_exe run

.PHONY: bmc
bmc: clean_bmc formal formal/bmc/$(DUT)/PASS
formal/bmc/$(DUT)/PASS: script/$(DUT)_bmc.sby rtl/$(DUT).v
	mkdir formal/bmc
	cp script/$(DUT)_bmc.sby formal/bmc/$(DUT).sby 
	time sby -f formal/bmc/$(DUT).sby

.PHONY: ki
ki: clean_ki formal formal/ki/$(DUT)/PASS
formal/ki/$(DUT)/PASS: script/$(DUT)_ki.sby rtl/$(DUT).v
	mkdir formal/ki
	cp script/$(DUT)_ki.sby formal/ki/$(DUT).sby 
	time sby -f formal/ki/$(DUT).sby

.PHONY: pdr
pdr: clean_pdr formal formal/pdr/$(DUT)/PASS
formal/pdr/$(DUT)/PASS: script/$(DUT)_pdr.sby rtl/$(DUT).v
	mkdir formal/pdr
	cp script/$(DUT)_pdr.sby formal/pdr/$(DUT).sby 
	time sby -f formal/pdr/$(DUT).sby

formal: formal
	mkdir formal

build/design_under_test.json: rtl/$(DUT).v
	yosys -ql build/design_under_test.log script/$(DUT).ys
	
build/design_under_test.v: rtl/$(DUT).v $(CRPG_PATH)include/template/assertion.hpp
	python3 $(CRPG_PATH)script/DUTPreprocessing.py \
			$(DUT) \
			rtl \
			build \
			build/assertion.hpp \
			$(CRPG_PATH)include/template/assertion.hpp

build/obj_dir/assertion.hpp: build/assertion.hpp
	mv build/assertion.hpp build/obj_dir/assertion.hpp

build/obj_dir/interface.hpp: build/obj_dir/Vdesign_under_test___024root.h \
							 build/design_under_test.json \
							 build/design_under_test.log \
							 $(CRPG_PATH)include/template/interface.hpp
	python3 $(CRPG_PATH)script/IOWrapper.py $(DUT) \
			build/obj_dir/Vdesign_under_test___024root.h \
			build/design_under_test.json \
			build/design_under_test.log \
			$(CRPG_PATH)include/template/interface.hpp \
			build/obj_dir/interface.hpp \
			$(CRPG_PATH)include/template/control_signal.hpp \
			build/obj_dir/control_signal.hpp

build/obj_dir/Vdesign_under_test.cpp: build/design_under_test.v
	#$(VERILATE) $(wildcard build/*.v)
	$(VERILATE) build/design_under_test.v

build/obj_dir/Vdesign_under_test: build/obj_dir/Vdesign_under_test.cpp \
								  build/obj_dir/Vdesign_under_test.h
	$(SUBMAKE) Vdesign_under_test.mk

build/Vdesign_under_test: build/obj_dir/Vdesign_under_test
	mv build/obj_dir/Vdesign_under_test build/Vdesign_under_test

init:
	@echo "#INFO Initializing build file";
	mkdir build
	#cp rtl/*.v build/

design_info: build/design_under_test.json build/design_under_test.log
	@echo "#INFO Extracting information from design";

design_pre: build/design_under_test.v
	@echo "#INFO Preparing Verilog for Verilator";

sim_rtl: build/obj_dir/Vdesign_under_test.cpp
	@echo "#INFO Generating C++ simulator code";

gen_include: build/obj_dir/assertion.hpp \
			 build/obj_dir/interface.hpp \
			 build/obj_dir/control_signal.hpp
	@echo "#INFO Generating c++ Header for Verilator";

sim_exe: build/obj_dir/Vdesign_under_test.cpp
	@echo "#INFO Building in C++ simulator";

run: build/Vdesign_under_test
	time ./build/Vdesign_under_test

clean: clean_formal clean_build

clean_build:
	rm -rf build

clean_formal:
	rm -rf formal

clean_bmc:
	rm -rf formal/bmc

clean_ki:
	rm -rf formal/ki

clean_pdr:
	rm -rf formal/pdr
