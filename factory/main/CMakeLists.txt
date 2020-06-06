
set(COMPONENT_SRCS "test.c")

set(COMPONENT_ADD_INCLUDEDIRS "")

# requirements can't depend on config
set(COMPONENT_REQUIRES espfs libesphttpd nvs_flash mcommon openssl)

set(COMPONENT_EMBED_TXTFILES certificate.der key.der)

register_component()
