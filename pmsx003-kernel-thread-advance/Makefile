obj-m += pmsx003.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	@echo "Building PMSX003 kernel module..."
	make -C $(KDIR) M=$(PWD) modules

clean:
	@echo "Cleaning PMSX003 kernel module..."
	make -C $(KDIR) M=$(PWD) clean

install:
	@echo "Installing PMSX003 kernel module..."
	sudo insmod pmsx003.ko
	sudo depmod -a

uninstall:
	@echo "Uninstalling PMSX003 kernel module..."
	sudo rmmod pmsx003

check:
	@echo "Running kernel module checks with lockdep..."
	make -C $(KDIR) M=$(PWD) C=1 CONFIG_DEBUG_LOCK_ALLOC=y CONFIG_PROVE_LOCKING=y CONFIG_DEBUG_FS=y

test:
	@echo "Running PMSX003 user-space test..."
	gcc -o pmsx003_user pmsx003_user.c
	./pmsx003_user

.PHONY: all clean install uninstall check test