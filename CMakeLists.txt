idf_component_register(
	SRCS "m90e26.c"
	INCLUDE_DIRS "."
	PRIV_INCLUDE_DIRS
	REQUIRES
	PRIV_REQUIRES commands endpoints hal_esp32 printfx rules ssd1306 syslog systiming stringsX values  
)
