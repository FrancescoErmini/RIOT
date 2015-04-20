
APPLICATION = my_project
BOARD ?= udoo
PORT= /dev/cu.SLAB_USBtoUART
RIOTBASE ?= $(CURDIR)/../..
QUIET ?= 1
#DISABLE_MODULE += auto_init

CFLAGS += -DDEVELHELP
CXXEXFLAGS +=
#featured required
FEATURES_PROVIDED += periph_uart periph_gpio periph_spi  cpp
#FEATURES_PROVIDED += periph_random
# Modules to include:
#USEMODULE += keypad4x4
USEMODULE += pn532
USEMODULE += keyexc
USEMODULE += xbee
USEMODULE += shell
USEMODULE += shell_commands
#USEMODULE += uart0
#USEMODULE += ps
#USEMODULE += random
USEMODULE += vtimer

include $(RIOTBASE)/Makefile.include
