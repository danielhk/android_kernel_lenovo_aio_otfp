include $(srctree)/drivers/misc/mediatek/Makefile.custom
obj-y += md32.o
md32-y := md32_helper.o
md32-y += md32_ipi.o md32_irq.o md32_ocd.o

ifeq ($(CONFIG_MT_ENG_BUILD),y)
CFLAGS_md32_helper.o += -DDEBUG
CFLAGS_md32_ipi.o += -DDEBUG
CFLAGS_md32_irq.o += -DDEBUG
CFLAGS_md32_ocd.o += -DDEBUG
endif