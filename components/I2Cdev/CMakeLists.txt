idf_component_register(
	SRCS "I2Cdev.cpp"
	PRIV_REQUIRES driver esp_driver_i2c
	INCLUDE_DIRS "."
)
