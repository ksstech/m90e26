set(srcs "m90e26.c" )
set(include_dirs "." )
#set(priv_include_dirs "." )
set(requires rtos-support )
set(priv_requires hal_esp32 endpoints ssd1306 rules common systiming syslog printf values stringsX )
idf_component_register( SRCS ${srcs}
						INCLUDE_DIRS ${include_dirs} 
#						PRIV_INCLUDE_DIRS ${priv_include_dirs} 
						REQUIRES ${requires} 
						PRIV_REQUIRES ${priv_requires}
						)
