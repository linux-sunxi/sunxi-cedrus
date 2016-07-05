KPATH ?= /build
KMAKE := $(MAKE) -C $(KPATH) M=$(CURDIR)/module

modules:
	$(KMAKE) modules
clean:
	$(KMAKE) clean

