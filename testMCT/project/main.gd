extends Node3D


# Called when the node enters the scene tree for the first time.
func _ready():
	print("MCT ready")
	TestMCT.start_xr()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	$GDExample/MeshInstance3D.rotate(Vector3.FORWARD, delta)
	print("MCT process")
	
