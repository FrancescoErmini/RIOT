APPLICATION = key_exchange
BOARD ?= udoo
PORT= /dev/cu.SLAB_USBtoUART
RIOTBASE ?= $(CURDIR)/../..
QUIET ?= 1
#DISABLE_MODULE += auto_init${cross_make} BOARD=${RIOT_BOARD} /examples/my_project/Makefile

FEATURES_REQUIRED = periph_uart periph_gpio

ifneq (,$(filter udoo,$(BOARD)))
  export XBEE_UART ?= UART_1
endif

USEMODULE += xbee
USEMODULE += ng_netif
USEMODULE += auto_init_ng_netif
USEMODULE += ng_nomac
USEMODULE += ng_pktdump
USEMODULE += shell
USEMODULE += shell_commands


ifneq (,$(XBEE_UART))
  CFLAGS += -DXBEE_UART=$(XBEE_UART)
else
  # set default
  CFLAGS += -DXBEE_UART=UART_0
endif

# add current directory to the include path. Putting it in CFLAGS will make
# it go to the beginning, before the standard includes.
# That way xbee_params.h get's included and auto configuration can pick it up.
CFLAGS += -I$(CURDIR)



include $(RIOTBASE)/Makefile.include