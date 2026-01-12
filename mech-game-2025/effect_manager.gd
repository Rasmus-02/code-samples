extends Node

var muzzleflash_medium = preload("res://Particle Systems/muzzleflash_medium.tscn")
var casing_40mm = preload("res://Particle Systems/casing_40mm.tscn")
var explosion_small = preload("res://Particle Systems/explosion_particle_small.tscn")
var root

func _process(delta: float) -> void:
	if Globals.scene == Globals.scenes.COMBAT:
		root = get_tree().root.get_node("Main").get_node("Network")
	else:
		root = get_tree().root

@rpc("any_peer", "call_local") # To spawn use for example:  rpc("spawn_particle_on_all", muzzleflash, position, rotation)
func spawn_particle_on_all(particle_path: String, position: Vector3, rotation: Vector3, parent = root):
	spawn_particle(particle_path, position, rotation, parent)

func spawn_particle(particle_path : String, position : Vector3, rotation : Vector3, parent = root):
	var particle : PackedScene
	match particle_path:
		"res://Particle Systems/muzzleflash_medium.tscn":
			particle = muzzleflash_medium
		"res://Particle Systems/casing_40mm.tscn":
			particle = casing_40mm
		"res://Particle Systems/explosion_particle_small.tscn":
			particle = explosion_small
	var instance = particle.instantiate()
	parent.add_child(instance)
	instance.global_position = position
	instance.global_rotation = rotation
