extends Node3D

@export var animation : AnimationPlayer = null
@export var animation_name : String = ""
@export var particle : GPUParticles3D = null


func _ready() -> void:
	if animation != null:
		animation.play(animation_name)
	elif particle != null:
		particle.emitting = true


# Delete particle when done

func _on_animation_player_animation_finished(_anim_name: StringName) -> void:
	queue_free()

func _on_gpu_particles_3d_finished() -> void:
	queue_free()
