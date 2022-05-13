# M90E26
set( srcs "m90e26.c" )
set( include_dirs "." )
set( requires common hal_esp32 rules nvs_flash )
set( priv_requires printfx rtos-support ssd1306 syslog systiming )
idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} PRIV_REQUIRES ${priv_requires})
# endpoints