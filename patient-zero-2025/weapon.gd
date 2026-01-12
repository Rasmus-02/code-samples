extends Node
class_name Weapon

##RPM
@export_range(1, 600, 1.0) var firerate : float = 60
@export_range(0, 5, 0.1) var aim_time : float = 0
@export var agent: Agent
@export var bullet : PackedScene
@export var muzzle_flash_location : Marker3D
@export_range(0.5, 1.0, 0.05) var accuracy  = 1.0
@onready var firerate_timer = $FirerateTimer
@onready var shoot_raycast: RayCast3D

var ready_to_fire : bool = true # For firerate
var aiming : bool = false # For aim timer logic (await would otherwise bug up the script)


func _ready() -> void:
	#Wait for parent to set up raycast
	await get_tree().create_timer(0.1).timeout
	shoot_raycast = agent.get_node("GunRayCast")
	# Set range
	shoot_raycast.target_position = Vector3(0,0, -100)
	# Set firerate
	firerate_timer.wait_time = 60.0 / firerate


func shoot() -> void:
	if ready_to_fire and is_instance_valid(shoot_raycast) and aiming == false:
		aiming = true
		
		# Shoot animation (and "pause animation" to imitate aiming)
		agent.animation_handler.play_animation("Attack Human")
		agent.animation_handler.set_animation_speed(0)
		
		# Aim at target
		await get_tree().create_timer(aim_time * 0.5).timeout
		
		# Handle firerate logic
		ready_to_fire = false
		
		# Keep aim at where target was (and set speed back to normal)
		await get_tree().create_timer(aim_time * 0.5).timeout
		agent.animation_handler.set_animation_speed(1)
		
		# Handle firerate logic
		firerate_timer.start()
		
		# Spawn muzzleflash + sound and go to recoil in animation
		agent.animation_player.seek(0.4, true)
		if muzzle_flash_location != null:
			EffectManager.spawn_particle(EffectManager.rifle_shoot, muzzle_flash_location.global_position, muzzle_flash_location.global_rotation)
			EffectManager.play_sound(EffectManager.rifle_sound, agent.global_position, 2)
		
		# Check if zombie in bullet path
		var target
		if shoot_raycast.is_colliding():
			target = shoot_raycast.get_collider()
			if target is Agent and agent.team != target.team:
				target.health -= agent.damage
		
		aiming = false


func _on_firerate_timer_timeout() -> void:
	ready_to_fire = true
	aiming = false
