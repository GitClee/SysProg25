idf_component_register(SRCS "main.c" 
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver bt nvs_flash
                    INCLUDE_DIRS "")
