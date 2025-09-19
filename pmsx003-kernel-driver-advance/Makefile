obj-m += pmsx003.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean

install:
	sudo insmod pmsx003.ko

uninstall:
	sudo rmmod pmsx003

check:
	@echo "Running kernel module checks..."
	@make -C $(KDIR) M=$(PWD) C=1

.PHONY: all clean install uninstall check