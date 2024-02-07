@tool
extends EditorPlugin

func _enter_tree():
	# Register our autoload object
	add_autoload_singleton(
			"TestMCT",
			"res://addons/TestMCT/TestMCT.gd")
