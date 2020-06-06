
set(COMPONENT_SRCS "test.c")

set(COMPONENT_ADD_INCLUDEDIRS "")

# requirements can't depend on config
set(COMPONENT_REQUIRES mcommon mconfig mwifi mlink mesh_mqtt_handle2)

register_component()
