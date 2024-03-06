extends Node3D

var _plugin_name = "ARCoreGDExtension"
var _android_plugin

# Called when the node enters the scene tree for the first time.
func _ready():
	if Engine.has_singleton(_plugin_name):
		_android_plugin = Engine.get_singleton(_plugin_name)
	else:
		printerr("Couldn't find plugin " + _plugin_name)
		
	_android_plugin.helloWorld()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass
