APPLICATION = key_exchange
BOARD ?= udoo
#PORT= /dev/cu.SLAB_USBtoUART
#PORT_DARWIN ?= $(shell ls -1 /dev/tty.SLAB_USBtoUART* | head -n 1)
RIOTBASE ?= $(CURDIR)/../..
QUIET ?= 1

FEATURES_PROVIDED += periph_uart periph_gpio periph_spi 
#FEATURES_PROVIDED += periph_random  cpp


BOARD_INSUFFICIENT_RAM := stm32f0discovery

# Define default pin mappings for some boards:
#ifneq (,$(filter udoo,$(BOARD)))
#  export XBEE_UART ?= UART_1
#endif
#SRC += $(RIOTBASE)/sys/shell/commands/sc_netif.c


USEMODULE += xbee
USEMODULE += gnrc_netif
USEMODULE += auto_init_gnrc_netif
USEMODULE += gnrc_nomac
USEMODULE += gnrc_pktdump
#USEMODULE += shell
#USEMODULE += shell_commands 
#USEMODULE += uart0
USEMODULE += vtimer
USEMODULE += pn532
USEMODULE += keyexc

# set default UART to use in case none was defined
XBEE_UART ?= "UART_NUMOF-1"

# export UART to params file
CFLAGS += -DXBEE_UART=$(XBEE_UART)


CFLAGS += -I$(CURDIR)

include $(RIOTBASE)/Makefile.include
