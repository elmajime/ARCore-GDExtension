extends Camera3D


# Called when the node enters the scene tree for the first time.
func _ready():
	var feed = CameraServer.get_feed(0)
	feed.set_active(true)
	
	var viewport = SubViewport.new()
	add_child(viewport)
	viewport.size = Vector2(640, 480)
	var cam_texture = CameraTexture.new()
	cam_texture.feed_id = feed.get_id()
	
	var sprite = Sprite2D.new()
	sprite.texture = cam_texture
	viewport.add_child(sprite)
	
	feed.camera_feed_updated.connect(_on_CameraFeed_updated)
	
func _on_CameraFeed_updated():
	var frame = CameraServer.feed_texture.get_data()
	# Process frame here

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass
