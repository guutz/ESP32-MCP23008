idf_component_register(SRCS "mcp23008.c" "mcp23008_wrapper.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES i2c_manager)
