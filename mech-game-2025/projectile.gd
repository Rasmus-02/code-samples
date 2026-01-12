extends RigidBody3D
class_name Projectile

@export_range(0,2000, 10) var speed : float = 0
@export_range(0,500, 0.1) var diameter : float = 100
@export_range(0.01,2, 0.01) var ap_modifier : float = 1
@export_range(0,20000, 1) var explosive_mass : float = 1
@export var cleanup_timer : Timer = null

var kinetic_energy : float = 0.0 # Gets calculated on collision
var penetration_power: float = 0.0 # Gets calculated on collision
var direction : Vector3 = Vector3.ZERO
var velocity_before_contact : Vector3
var collided : bool = false
var collision_angle : float = 0.0


func shoot() -> void:
	linear_velocity = direction * speed
	velocity_before_contact = linear_velocity
	bullet_trail()

func deal_damage(_body) -> void:
	pass

func bullet_trail():
	pass

func _integrate_forces(state):
	if state.get_contact_count() > 0:
		var normal = state.get_contact_local_normal(0)  # Normal of the first contact point
		var velocity_dir = velocity_before_contact.normalized()  # Direction of movement
		var angle_radians = abs(acos(velocity_dir.dot(normal)))  # Compute the angle in radians
		var angle_degrees = rad_to_deg(angle_radians)  # Convert to degrees
		collision_angle = abs(angle_degrees - 180)


func _process(_delta: float) -> void:
	if collided == false:
		if velocity_before_contact.length() > linear_velocity.length() * 1.1:
			collided = true
	if collided == false:
		velocity_before_contact = linear_velocity
	elif cleanup_timer != null and cleanup_timer.is_stopped() == true:
		cleanup_timer.start()

func delete():
	if is_multiplayer_authority():
		queue_free()

# Helpers
func calculate_kinetic_energy():
	return 0.5 * mass * velocity_before_contact.length() ** 2 # (1/2)mv^2
func calculate_penetration_power():
	return (kinetic_energy * ap_modifier) / Globals.armor_toughness
