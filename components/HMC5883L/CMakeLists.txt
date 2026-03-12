idf_component_register(
	SRCS "HMC5883L.cpp"
	INCLUDE_DIRS "."
	PRIV_REQUIRES driver esp_driver_i2c
	REQUIRES I2Cdev
)
