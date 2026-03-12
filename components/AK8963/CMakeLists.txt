idf_component_register(
	SRCS "AK8963.cpp"
	INCLUDE_DIRS "."
	PRIV_REQUIRES driver esp_driver_i2c
	REQUIRES I2Cdev
)
