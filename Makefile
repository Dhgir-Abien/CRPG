################################################################################
##
## Filename: Makefile
## Project:	 Constrained Random Pattern Generation
##
## Creator:	 Dhgir.Abine@gmail.com
##
################################################################################

MAKE := make
SUBMAKE := $(MAKE) -w -C
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
export CRPG_PATH := $(dir $(MKFILE_PATH))

.PHONY: case%
case%:
	@echo "Building $@ for Verilator";
	+@$(SUBMAKE) test/$@/
