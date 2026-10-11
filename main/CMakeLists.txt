idf_component_register(SRCS "hello_world_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES 
                            driver 
                            freertos 
                            esp_rom 
                            mqtt 
                            esp_wifi 
                            esp_event 
                            nvs_flash 
                            esp_netif
                            esp_hw_support)