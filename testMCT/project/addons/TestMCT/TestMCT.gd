@tool
extends Node

var ar_interface : ARCoreInterface

func get_interface():
	return ar_interface

func start_xr():
	if ar_interface:
		print("Capabilities " + str(ar_interface.get_capabilities()))
		print("Target size " + str(ar_interface.get_render_target_size()))

		if ar_interface.initialize():
			get_viewport().use_xr = true

			print("Initialised")
		else:
			print("Failed to initialise")
	else:
		print("Interface was not instantiated")


func _enter_tree():
	ar_interface = ARCoreInterface.new()
	if ar_interface:
		XRServer.add_interface(ar_interface)


func _exit_tree():
	if ar_interface:
		XRServer.remove_interface(ar_interface)
		ar_interface = null
