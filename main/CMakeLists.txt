set(COMPONENT_SRCS "ble_config.c"
                   "radiodevice.c"                                                        
                   "OWIcrc.c"
                   "OWIHighLevelFunctions.c"
                   "OWISWBitFunctions.c")

set(COMPONENT_ADD_INCLUDEDIRS ".;./include")

register_component()
