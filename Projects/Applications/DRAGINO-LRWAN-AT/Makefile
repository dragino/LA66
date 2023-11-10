
PROJECT := $(notdir $(CURDIR))

$(PROJECT)_SOURCE := $(wildcard src/*.c)  \
    $(TREMO_SDK_PATH)/Drivers/system/printf-stdarg.c  \
    $(TREMO_SDK_PATH)/Drivers/system/system_cm4.c  \
    $(TREMO_SDK_PATH)/Drivers/system/startup_cm4.S \
    $(wildcard $(TREMO_SDK_PATH)/Drivers/peripheral/src/*.c)  \
	$(wildcard $(TREMO_SDK_PATH)/Drivers/sensor/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/Middlewares/LoRa/system/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/Middlewares/LoRa/system/Crypto/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/Middlewares/LoRa/radio/sx126x/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/Middlewares/LoRa/driver/*.c) \
    $(wildcard $(TREMO_SDK_PATH)/Middlewares/LoRa/LoRaMac-node/region/*.c) \
	$(wildcard $(TREMO_SDK_PATH)/Middlewares/LoRa/LoRaMac-node/*.c) \
    $(wildcard $(TREMO_SDK_PATH)/Middlewares/LoRa/Core/*.c)

$(PROJECT)_INC_PATH := inc \
    $(TREMO_SDK_PATH)/Drivers/CMSIS \
	$(TREMO_SDK_PATH)/Drivers/crypto/inc \
	$(TREMO_SDK_PATH)/Drivers/peripheral/inc \
	$(TREMO_SDK_PATH)/Drivers/sensor \
    $(TREMO_SDK_PATH)/Drivers/system \
	$(TREMO_SDK_PATH)/Middlewares/LoRa/Core \
    $(TREMO_SDK_PATH)/Middlewares/LoRa/driver \
    $(TREMO_SDK_PATH)/Middlewares/LoRa/LoRaMac-node \
    $(TREMO_SDK_PATH)/Middlewares/LoRa/LoRaMac-node/region \
    $(TREMO_SDK_PATH)/Middlewares/LoRa/radio \
    $(TREMO_SDK_PATH)/Middlewares/LoRa/radio/sx126x \
	$(TREMO_SDK_PATH)/Middlewares/LoRa/system \
    $(TREMO_SDK_PATH)/Middlewares/LoRa/system/Crypto \
    $(TREMO_SDK_PATH)/Projects/Applications/DRAGINO-LRWAN-AT/inc
	
$(PROJECT)_CFLAGS  := -Wall -Os -ffunction-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fsingle-precision-constant -std=gnu99 -fno-builtin-printf -fno-builtin-sprintf -fno-builtin-snprintf
$(PROJECT)_DEFINES := -DCONFIG_DEBUG_UART=UART0 -DREGION_EU868 -DCONFIG_LOG -DPRINT_BY_DMA -DCONFIG_LORA_USE_TCXO -DDRAGINO_LA66

$(PROJECT)_LDFLAGS := -Wl,--gc-sections -Wl,--wrap=printf -Wl,--wrap=sprintf -Wl,--wrap=snprintf

$(PROJECT)_LINK_LD := cfg/gcc.ld

# please change the settings to download the app
SERIAL_PORT        :=/dev/ttyS10
#SERIAL_BAUDRATE    :=
#$(PROJECT)_ADDRESS :=

##################################################################################################
include $(TREMO_SDK_PATH)/build/make/common.mk



