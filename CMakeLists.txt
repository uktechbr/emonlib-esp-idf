
set(COMPONENT_SRCS "emonlib-esp-idf.c")

set(COMPONENT_INCLUDEDIRS "include")

idf_component_register(SRCS "${COMPONENT_SRCS}"
                    INCLUDE_DIRS "${COMPONENT_INCLUDEDIRS}"
                    REQUIRES esp_adc_cal esp_timer log)
