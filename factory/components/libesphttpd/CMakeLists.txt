set (COMPONENT_SRCS "core/auth.c" "core/httpd-freertos.c" "core/httpd.c" "core/sha1.c" "core/libesphttpd_base64.c" "util/captdns.c" "util/cgiflash.c" "util/cgiredirect.c" "util/cgiwebsocket.c" "util/cgiwifi.c" "util/cgiredirect.c" "util/esp32_flash.c" "util/esp32_httpd_vfs.c")

set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES app_update json spi_flash wpa_supplicant openssl)

set (COMPONENT_PRIV_INCLUDEDIRS "core"
                                "util")
if (CONFIG_ESPHTTPD_USE_ESPFS)
   list (APPEND COMPONENT_SRCS "core/httpd-espfs.c")
   list (APPEND COMPONENT_PRIV_INCLUDEDIRS "../espfs/include")
endif (CONFIG_ESPHTTPD_USE_ESPFS)

register_component()

target_compile_definitions (${COMPONENT_TARGET} PUBLIC -DFREERTOS)
