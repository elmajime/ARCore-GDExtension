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

func enable_depth_estimation(enable):
	if arcore_interface:
		arcore_interface.enable_depth_estimation(enable)

func show_depth_map(enable):
	if arcore_interface:
		arcore_interface.show_depth_map(enable)

func set_depth_color_mapping(mid_depth_meters, max_depth_meters):
	if arcore_interface:
		arcore_interface.set_depth_color_mapping(mid_depth_meters, max_depth_meters)

func enable_plane_detection(enable):
	if arcore_interface:
		arcore_interface.enable_plane_detection(enable)
		
func _enter_tree():
	arcore_interface = ARCoreInterface.new()
	if arcore_interface:
		XRServer.add_interface(arcore_interface)


func _exit_tree():
	if arcore_interface:
		XRServer.remove_interface(arcore_interface)
		arcore_interface = null
