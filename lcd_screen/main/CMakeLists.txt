idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_lcd
                    EMBED_FILES image.jpg)
