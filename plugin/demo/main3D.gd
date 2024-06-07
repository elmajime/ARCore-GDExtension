extends Node3D

var _plugin_name = "ARCoreGDExtension"
var _android_plugin

# Called when the node enters the scene tree for the first time.
func _ready():
	for s in Engine.get_singleton_list():
		print("MCT " + s.get_basename())
	if Engine.has_singleton(_plugin_name):
		print("MCT found plugin")
		_android_plugin = Engine.get_singleton(_plugin_name)
	else:
		printerr("MCT Couldn't find plugin " + _plugin_name)
		
	print("MCT before initialize_wrapper")
	_android_plugin.helloWorld()
	_android_plugin.initializeWrapper()
	
	print("MCT called initialize_wrapper")

	var ar_interface = ARCoreGDExtension.get_interface()
	var interface_name = ar_interface.get_name()
	print("MCT " + interface_name)
	
	ARCoreGDExtension.start_ar()


	#ARCoreGDExtension.enable_depth_estimation(true)
	#ARCoreGDExtension.show_depth_map(true)
	#ARCoreGDExtension.enable_plane_detection(true)

	#ARCoreGDExtension.set_depth_color_mapping(2.0, 65.0)
	#ARCoreGDExtension.set_depth_color_mapping(1.0, 1.0)
	

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	var screen_size = DisplayServer.screen_get_size()
	print("MCT_screen_size" + str(screen_size.x) + ", " + str(screen_size.y))


func _on_node_2d_estimate_depthmap_toggled(toggled):
	ARCoreGDExtension.enable_depth_estimation(toggled)

func _on_node_2d_show_depthmap_toggled(toggled):
	ARCoreGDExtension.show_depth_map(toggled)
	
	
func _on_node_2d_far_changed(value):
	ARCoreGDExtension.set_depth_color_mapping(1.0, value)


func _on_node_2d_planes_toggled(toggled):
	ARCoreGDExtension.enable_plane_detection(toggled)

