### Environment constants 

ARCH ?=
CROSS_COMPILE ?= arm-linux-gnueabihf-
export

### general build targets

all:
	$(MAKE) all -e -C libloragw
	$(MAKE) all -e -C util_pkt_fwd

clean:
	$(MAKE) clean -e -C libloragw
	$(MAKE) clean -e -C util_pkt_fwd

### EOF
