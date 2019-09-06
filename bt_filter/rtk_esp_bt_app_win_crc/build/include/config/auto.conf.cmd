deps_config := \
	/c/esp32/bt_filter/rtk_esp_bt_app_win_crc/aceinna_app/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/app_trace/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/aws_iot/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/bt/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/driver/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/esp32/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/esp_adc_cal/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/esp_http_client/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/ethernet/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/fatfs/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/freertos/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/heap/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/libsodium/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/log/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/lwip/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/mbedtls/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/nvs_flash/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/openssl/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/pthread/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/spi_flash/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/spiffs/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/tcpip_adapter/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/vfs/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/wear_levelling/Kconfig \
	/d/toolchain/esp/esp-idf-v3.1.3/components/bootloader/Kconfig.projbuild \
	/d/toolchain/esp/esp-idf-v3.1.3/components/esptool_py/Kconfig.projbuild \
	/d/toolchain/esp/esp-idf-v3.1.3/components/partition_table/Kconfig.projbuild \
	/d/toolchain/esp/esp-idf-v3.1.3/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
