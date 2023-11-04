idf_component_register(SRCS "main.cpp" "mpu6050.cpp" "wifi.c" "udp_trans.c" "web_server.c" "web_client.c"
    INCLUDE_DIRS "."
    EMBED_FILES "../html/error.html"
                "../html/favicon.ico"
                "../html/main.js"
                "../html/root.html"
                "../html/main.css")
