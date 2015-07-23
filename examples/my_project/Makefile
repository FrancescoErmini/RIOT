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
ifneq (,$(filter stm32f0discovery,$(BOARD)))
  export XBEE_UART ?= UART_1
endif
ifneq (,$(filter stm32f4discovery,$(BOARD)))
  export XBEE_UART ?= UART_1
endif
ifneq (,$(filter nucleo-f091,$(BOARD)))
  export XBEE_UART ?= UART_1
endif
ifneq (,$(filter udoo,$(BOARD)))
  export XBEE_UART ?= UART_1
endif
#SRC += $(RIOTBASE)/sys/shell/commands/sc_netif.c


USEMODULE += xbee
USEMODULE += ng_netif
USEMODULE += auto_init_ng_netif
USEMODULE += ng_nomac
USEMODULE += ng_pktdump
#USEMODULE += shell
#USEMODULE += shell_commands 
USEMODULE += uart0
USEMODULE += vtimer
USEMODULE += pn532
USEMODULE += keyexc


ifneq (,$(XBEE_UART))
  CFLAGS += -DXBEE_UART=$(XBEE_UART)
 # CFLAGS += -DDEVELHELP

else
  # set default
  CFLAGS += -DXBEE_UART=UART_0
endif

CFLAGS += -I$(CURDIR)

include $(RIOTBASE)/Makefile.include