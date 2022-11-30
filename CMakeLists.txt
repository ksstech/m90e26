# M90E26

set( srcs "m90e26.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires common hal_esp32 nvs_flash rules ssd1306 )
set( priv_requires printfx rtos-support syslog systiming )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
