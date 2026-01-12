extends Marker3D

var yaw : float = 0.0
var pitch : float = 0.0
var aim_position : Vector3 = Vector3.ZERO
var zoom : float = 1.0
@onready var aim_ray = $Camera3D/AimRay
@onready var camera = $Camera3D
@onready var root = $".."

func _input(event):
	if event is InputEventMouseMotion:
		yaw -= event.relative.x * 0.1  # Adjust sensitivity
		pitch = clamp(pitch - event.relative.y * 0.1, -30, 30)  # Camera pitch range

func _process(delta: float) -> void:
	if not root.is_multiplayer_authority(): # Makes sure only correct player controlls certain mech
		set_process(false)  # Disable processing for non-owned mechs
		set_physics_process(false)  # Disable physics processing too
	else:
		camera.make_current()
	
	# Rotate Camera Pivot
	self.rotation_degrees.y = fmod(yaw, 360)
	self.rotation_degrees.x = fmod(pitch, 360)
	# Update aim position (weapons will point towards)
	if aim_ray.is_colliding():
		aim_position = aim_ray.get_collision_point()
	
	if Input.is_action_pressed("Zoom"):
		zoom = lerp(zoom, 4.0, delta * 5)
	else:
		zoom = lerp(zoom, 1.0, delta * 10)
	camera.fov = 75 / zoom
