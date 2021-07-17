set(component_srcs "src/u8g2_esp32_hal.cpp")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "u8g2"
                       REQUIRES "")

