extends Node3D

@onready var raycast = $"Collide raycast"

func _ready() -> void:
	var tween = get_tree().create_tween()
	tween.tween_property(get_child(0), "scale", Vector3(1,1,1), 2)
	EffectManager.play_sound(EffectManager.mortar_falling_sound, global_position * Vector3(1,0,1))

func _physics_process(_delta: float) -> void:
	if raycast.is_colliding():
		explode()

func explode() -> void:
	EffectManager.spawn_particle(EffectManager.explosion, self.global_position)
	EffectManager.play_sound(EffectManager.explosion_sound, global_position, 3)
	queue_free()
