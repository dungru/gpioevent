obj-m += gpioevent.o

INCLUDE += -I$(KDIR)/arch/arm/mach-omap2/include
INCLUDE += -I$(KDIR)/arch/arm/plat-omap/include

CFLAGS_gpioevent.o ?= -DEVENT_PIN=2

all:
	$(MAKE) -C $(KDIR) $(INCLUDE) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) $(INCLUDE) M=$(PWD) clean

