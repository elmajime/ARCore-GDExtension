@tool
extends Node

var arcore_interface : ARCoreInterface

func get_interface():
	return arcore_interface

func start_ar():
	if arcore_interface:
		print("Capabilities " + str(arcore_interface.get_capabilities()))
		print("Target size " + str(arcore_interface.get_render_target_size()))

		if arcore_interface.initialize():
			get_viewport().use_xr = true

			print("Initialised")
		else:
			print("Failed to initialise")
	else:
		print("Interface was not instantiated")


func _enter_tree():
	arcore_interface = ARCoreInterface.new()
	if arcore_interface:
		XRServer.add_interface(arcore_interface)


func _exit_tree():
	if arcore_interface:
		XRServer.remove_interface(arcore_interface)
		arcore_interface = null
