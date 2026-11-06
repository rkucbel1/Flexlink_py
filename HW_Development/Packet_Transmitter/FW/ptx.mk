# ptx.mk - Pluto transmitter server (ptx_server). Use: make -f ptx.mk [target]
# (rename it to Makefile to drop the -f)
#   make -f ptx.mk         PC build: ptx_server_stub (the PL replaced by a file and its timing) and
#                          ptx_params_test; then: python test_fw.py [--rtl]
#   make -f ptx.mk pluto FW=~/plutosdr-fw
#                          ARM build with the plutosdr-fw buildroot toolchain and sysroot (libiio
#                          comes with it); or give CROSS= and SYSROOT= yourself
#   make -f ptx.mk deploy run PLUTO=192.168.2.1
#                          copy it to the Pluto's /tmp (RAM: copy again after a power cycle) and
#                          start it (ssh root, password analog)
FW ?= $(HOME)/plutosdr-fw
# the buildroot cross compiler, whatever its prefix (arm-none-linux-gnueabihf-, arm-linux-gnueabihf-, ...)
CROSS_GCC := $(firstword $(wildcard $(FW)/buildroot/output/host/bin/arm*-linux-gnueabihf-gcc))
CROSS ?= $(CROSS_GCC:gcc=)
SYSROOT ?= $(FW)/buildroot/output/staging
PLUTO ?= 192.168.2.1
CFLAGS = -O2 -Wall -Wextra -std=gnu99

all: ptx_server_stub ptx_params_test

ptx_server_stub: ptx_server.c ptx_params.c ptx_params.h
	$(CC) $(CFLAGS) -DPTX_STUB -o $@ ptx_server.c ptx_params.c -lpthread -lm

ptx_params_test: ptx_params_test.c ptx_params.c ptx_params.h
	$(CC) $(CFLAGS) -o $@ ptx_params_test.c ptx_params.c -lm

pluto: ptx_server

ptx_server: ptx_server.c ptx_params.c ptx_params.h
	@test -n "$(CROSS)" || { echo "no cross compiler in $(FW)/buildroot/output/host/bin: give CROSS=<prefix>"; exit 1; }
	$(CROSS)gcc $(CFLAGS) --sysroot=$(SYSROOT) -o $@ ptx_server.c ptx_params.c -liio -lpthread -lm

deploy: ptx_server
	cat ptx_server | ssh root@$(PLUTO) 'cat > /tmp/ptx_server.new && chmod +x /tmp/ptx_server.new && mv -f /tmp/ptx_server.new /tmp/ptx_server'

run:
	ssh root@$(PLUTO) 'killall ptx_server 2>/dev/null; nohup /tmp/ptx_server > /tmp/ptx_server.log 2>&1 &'

clean:
	rm -f ptx_server ptx_server_stub ptx_params_test ptx_stub_stream.bin
