@tool
extends EditorExportPlugin

# TODO: Update to your plugin's name.
var _plugin_name = "ARCoreGDExtension"
var _plugin_version = "1.0"

func _supports_platform(platform):
	if platform is EditorExportPlatformAndroid:
		return true
	return false

func _get_android_libraries(platform, debug):
	if debug:
		return PackedStringArray([_plugin_name + "/.bin/debug/arm64-v8a/" + _plugin_name + "-" + _plugin_version + "-debug.aar"])
	else:
		return PackedStringArray([_plugin_name + "/.bin/release/arm64-v8a/" + _plugin_name + "-" + _plugin_version + "-release.aar"])

func _get_name():
	return _plugin_name
