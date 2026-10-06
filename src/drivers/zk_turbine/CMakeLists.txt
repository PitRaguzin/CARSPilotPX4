
px4_add_module(
	MODULE drivers__zk_turbine
	MAIN zk_turbine
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		ZKTurbine.cpp
	MODULE_CONFIG
		module.yaml
)
