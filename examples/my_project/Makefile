APPLICATION = driver_xbee
BOARD ?= udoo
PORT= /dev/cu.SLAB_USBtoUART
RIOTBASE ?= $(CURDIR)/../..
QUIET ?= 1
#DISABLE_MODULE += auto_init${cross_make} BOARD=${RIOT_BOARD} /examples/my_project/Makefile

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
SRC += $(RIOTBASE)/sys/shell/commands/sc_netif.c

USEMODULE += xbee
#USEMODULE += pn532
#USEMODULE += uart0
USEMODULE += ng_netbase
USEMODULE += ng_nomac
USEMODULE += ng_pktdump
#USEMODULE += ng_netif
USEMODULE += shell
USEMODULE += shell_commands 
#USEMODULE += ng_netif
#USEMODULE += sixlowpan
#USEMODULE += net_if
#USEMODULE += ps
#USE_MODUE += random
#USEMODULE += vtimer
USEMODULE += pn532
USEMODULE += keyexc

ifneq (,$(XBEE_UART))
  CFLAGS += -DXBEE_UART=$(XBEE_UART)
else
  # set default
  CFLAGS += -DXBEE_UART=UART_0
endif


include $(RIOTBASE)/Makefile.include