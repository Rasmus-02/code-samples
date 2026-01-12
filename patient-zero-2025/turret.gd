extends Weapon

@export_enum("Arched", "Direct") var firemode : String
@export var turret : MeshInstance3D
@export var barrel : MeshInstance3D


func _ready() -> void:
	# Set firerate
	firerate_timer.wait_time = 60.0 / firerate

func shoot():
	if ready_to_fire:
		# Firerate logic
		ready_to_fire = false
		firerate_timer.start()
		# Mortar
		if firemode == "Arched":
			if muzzle_flash_location != null:
				EffectManager.spawn_particle(EffectManager.mortar_shoot, muzzle_flash_location.global_position, muzzle_flash_location.global_rotation)
				EffectManager.play_sound(EffectManager.mortar_shoot_sound, muzzle_flash_location.global_position, 3)
			var bullet_instance = bullet.instantiate()
			get_tree().root.add_child(bullet_instance)
			# Calculate where target will be when bullet drops
			var target_velocity = agent.target.velocity
			var bullet_travel_time = 2.7 # Estimated time for bullet to drop
			var aim_position = agent.target.global_position + target_velocity * bullet_travel_time
			# Add a bit of inaccuracy
			var target_distance = agent.global_position.distance_to(agent.target.global_position)
			var formatted_accuracy = (1 - accuracy)
			var rng_x = randi_range(-formatted_accuracy * target_distance, formatted_accuracy * target_distance)
			var rng_z = randi_range(-formatted_accuracy * target_distance, formatted_accuracy * target_distance)
			var inaccuracy = Vector3(rng_x, 0, rng_z)
			# Set bullet to correct position
			bullet_instance.global_position = aim_position + (Vector3.UP * 35) + inaccuracy

func _physics_process(_delta: float) -> void:
	if agent != null and agent.target != null and agent.health > 0:
		# Rotate turret on x and z axis
		var flat_target = Vector3(agent.target.global_position.x, turret.global_position.y, agent.target.global_position.z)
		turret.look_at(flat_target, Vector3.UP)
		# Rotate barrel on y axis
		var target_distance = turret.global_position.distance_to(agent.target.global_position)
		var normalized_distance = target_distance / agent.attack_range
		var barrel_angle = 115 - normalized_distance * 45
		barrel_angle = clamp(barrel_angle, 60, 85)
		barrel.rotation.x = deg_to_rad(barrel_angle)
		
