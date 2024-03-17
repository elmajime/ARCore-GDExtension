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


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass
