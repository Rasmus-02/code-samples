extends AnimatableBody3D

@export var drag = 0.02 # Air and ground friction
@export var track : PathFollow3D
@export var player_track : PathFollow3D
@export var offset : int = 2259 # how far player train should be - how far this train is
@export var enabled : bool = false


var speed : float = 0 # Kmh
var pressure : float = 10
var acceleration : float
var real_acceleration : float = 0.0

@onready var main: Node = $".."

func _physics_process(delta: float) -> void:
	if enabled == true:
		var throttle = 1
		if player_track.progress >= track.progress + offset:
			throttle = 1
		else:
			throttle = 0
		
		#Calculate acceleration from pressure
		if speed <= throttle * 100:
			acceleration = pressure * 5
		else:
			acceleration = -5
		
		#Calculate drag and brakes
		acceleration -= drag * (speed ** 2)
		real_acceleration = lerp(real_acceleration, acceleration, 1)
		
		#Apply acceleration and drag
		speed += real_acceleration * delta
		speed = clamp(speed, 0, 101)
		
		#Move the train
		track.progress += speed * 0.2777 * delta
