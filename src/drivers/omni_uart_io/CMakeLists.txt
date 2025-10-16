
px4_add_module(
	MODULE drivers__omni_uart_io
        MAIN omni_uart_interface
    	COMPILE_FLAGS
        -Os
    	SRCS
	    omni_uart_io.cpp

    	DEPENDS
	MODULE_CONFIG
	    module.yaml
)
