idf_component_register(SRCS "AK8963.cpp"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES driver
                      REQUIRES I2Cdev)
