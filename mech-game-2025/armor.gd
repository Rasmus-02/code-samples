extends StaticBody3D
class_name Armor

@export_range(0, 500, 10.0) var armor : float = 10
@export var part_specific : bool = false
@export var health_node : Node3D

func damage(amount : int):
	if health_node != null:
		var part = Part
		if part_specific: # If damage should be applied to specific body part
			part = get_parent()
		health_node.damage(amount, part)
