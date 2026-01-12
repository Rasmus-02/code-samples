extends Area3D

@export var damage : float
@export var one_shot : bool = false
@export var target_humans : bool = true
@export var target_infected : bool = true

func _on_body_entered(_body: Node3D) -> void:
	if one_shot:
		for bodies in get_overlapping_bodies():
			if (bodies.is_in_group("Human") and target_humans) or (bodies.is_in_group("Infected") and target_infected):
				do_damage(bodies)
		queue_free()

func _physics_process(_delta: float) -> void:
	if !one_shot and monitoring:
		for bodies in get_overlapping_bodies():
			if (bodies.is_in_group("Human") and target_humans) or (bodies.is_in_group("Infected") and target_infected):
				do_damage(bodies)

func do_damage(bodies: Node3D) -> void:
	bodies.health -= damage
	if !target_infected:
		bodies.has_been_infected = true
