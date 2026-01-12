extends AIController3D

var move : Vector2 = Vector2.ZERO
@onready var rays = [
	$"../RayFront",
	$"../RayLeft",
	$"../RayRight",
	$"../RayBack",
]
@onready var path = $"../../Path3D"

func get_obs() -> Dictionary:
	var ray_dists = []
	for ray in rays:
		if ray.is_colliding():
			ray_dists.append(ray.get_collision_point().distance_to(ray.global_position) / 10.0)
		else:
			ray_dists.append(1.0) # max vision range
	
	var curve : Curve3D = path.curve
	var origin = get_parent().global_transform.origin
	
	var closest_offset = curve.get_closest_offset(origin)
	var total_length = curve.get_baked_length()
	var progress = closest_offset / total_length
	
	var step_ratio = 0.02
	var next_offset = clamp(closest_offset + step_ratio * total_length, 0.0, total_length)
	var next_point = curve.sample_baked(next_offset)
	var deviation = origin - next_point
	var deviation_dir = deviation.normalized()
	
	var next_offset2 = clamp(closest_offset + (step_ratio * 2) * total_length, 0.0, total_length)
	var next_point2 = curve.sample_baked(next_offset2)
	var deviation2 = origin - next_point2
	var deviation_dir2 = deviation2.normalized()
	
	var next_offset3 = clamp(closest_offset + (step_ratio * 3) * total_length, 0.0, total_length)
	var next_point3 = curve.sample_baked(next_offset3)
	var deviation3 = origin - next_point3
	var deviation_dir3 = deviation3.normalized()
	
	var tangent = (curve.sample_baked(next_offset + 0.1) - next_point).normalized()
	
	var agent_velocity = get_parent().velocity.normalized()
	
	return {"obs": ray_dists + [
		progress,
		deviation_dir.x,
		deviation_dir.y,
		deviation_dir2.x,
		deviation_dir2.y,
		deviation_dir3.x,
		deviation_dir3.y,
		agent_velocity.x,
		agent_velocity.y,
		tangent.x,
		tangent.y
	]}

func get_reward() -> float:	
	return reward
	
func get_action_space() -> Dictionary:
	return {
		"move" : {
			"size": 2,
			"action_type": "continuous"
		},
	}
	
func set_action(action) -> void:	
	move.x = action["move"][0]
	move.y = action["move"][1]
