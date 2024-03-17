extends Sprite2D

# Called when the node enters the scene tree for the first time.
func _ready():
	print("MCT : cameras: ")
	for feed in CameraServer.feeds():
		print("MCT : " + feed.get_name())
		
	printt("MCT : no more cameras")

