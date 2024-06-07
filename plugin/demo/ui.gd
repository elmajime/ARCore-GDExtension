extends Control

signal estimate_depthmap_toggled(toggled)
signal show_depthmap_toggled(toggled)
signal planes_toggled(toggled)
signal far_changed(value)

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass


func _on_estimate_depthmap_toggled(toggled_on):
	estimate_depthmap_toggled.emit(toggled_on)


func _on_schow_depthmap_toggled(toggled_on):
	show_depthmap_toggled.emit(toggled_on)


func _on_plane_dection_toggled(toggled_on):
	planes_toggled.emit(toggled_on)


func _on_far_slider_value_changed(value):
	$Scale/FarValue.text = str(value)
	far_changed.emit(value)
