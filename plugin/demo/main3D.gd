extends Node3D

var _plugin_name = "ARCoreGDExtension"
var _android_plugin

var is_estimating_light = false
var original_light_trsf: Transform3D;

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
	_android_plugin.initializeWrapper()
	
	print("MCT called initialize_wrapper")

	var ar_interface = ARCoreGDExtension.get_interface()
	var interface_name = ar_interface.get_name()
	print("MCT " + interface_name)
	
	ARCoreGDExtension.start_ar()
	
	original_light_trsf = $DirectionalLight3D.transform

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	if (is_estimating_light):
		var light_dir = ARCoreGDExtension.get_light_main_hdr_direction()
		$DirectionalLight3D.rotation = light_dir
		var intensity = ARCoreGDExtension.get_light_main_hdr_intensity()
		$DirectionalLight3D.light_intensity_lumens = intensity.x;
	else:
		$DirectionalLight3D.transform = original_light_trsf


func _on_node_2d_estimate_depthmap_toggled(toggled):
	ARCoreGDExtension.enable_depth_estimation(toggled)

func _on_node_2d_show_depthmap_toggled(toggled):
	ARCoreGDExtension.show_depth_map(toggled)
	
func _on_node_2d_far_changed(value):
	ARCoreGDExtension.set_max_depth_meters(value)

func _on_node_2d_vertical_planes_toggled(toggled):
	ARCoreGDExtension.enable_vertical_plane_detection(toggled)

func _on_node_2d_horizontal_planes_toggled(toggled):
	ARCoreGDExtension.enable_horizontal_plane_detection(toggled)
	
func _on_node_2d_images_detection_toggled(toggled):
	ARCoreGDExtension.enable_images_detection(toggled)

func _on_node_2d_instant_placement_toggled(toggled):
	ARCoreGDExtension.enable_instant_placement(toggled)

func _on_node_2d_light_estimation_toggled(toggled):
	is_estimating_light = toggled
	ARCoreGDExtension.enable_light_estimation(toggled)

func _on_node_2d_point_cloud_detection_toggled(toggled):
	ARCoreGDExtension.enable_point_cloud_detection(toggled)

func _on_node_2d_switch_orientation(vertical):
	ARCoreGDExtension.switch_orientation(vertical)
