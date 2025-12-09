set(srcs "main.c")

idf_component_register(SRCS ${srcs}
                    REQUIRES driver
                    INCLUDE_DIRS ".")
