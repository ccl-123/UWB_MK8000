idf_component_register(SRCS "uwb_mk8000_core.c" "uwb_mk8000_at.c" "uwb_mk8000_uart.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver log freertos)
