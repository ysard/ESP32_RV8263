idf_component_register(
    SRCS "RV8263.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_timer i2c_manager
)
