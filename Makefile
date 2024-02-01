obj-m += ademco.o
ccflags-y := -Wimplicit-fallthrough=3 -DSOFT_UART=17
