@tool
extends AnimatableBody3D

var separation: float = 9.5

@export var following: PathFollow3D
@export var track: PathFollow3D

@export var locomotive: AnimatableBody3D

@onready var animation_player: AnimationPlayer = $AnimationPlayer

func _physics_process(_delta: float) -> void:
	track.progress = following.progress - separation
	if is_instance_valid(locomotive):
		rotate_wheels()
	else:
		queue_free()

func rotate_wheels() -> void:
	if !Engine.is_editor_hint():
		animation_player.speed_scale = locomotive.speed * 0.2777 / 4.14
