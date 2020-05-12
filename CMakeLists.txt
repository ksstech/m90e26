idf_component_register(	SRCS m90e26.c
						INCLUDE_DIRS . 
						REQUIRES rtos-support
						PRIV_REQUIRES hal_esp32 endpoints ssd1306 rules common systiming syslog printf values strings hal_esp32 rtos-support
						)
