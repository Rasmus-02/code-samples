extends Interactable

@export var valve_position : float = 0.0
@export var valve_min : float = 0.0
@export var valve_max : float = 1.0
@export var step : float = 0.05
@export var angle_range : float = 100
@export var label : Label3D = null
@export_enum("valve", "lever") var type = "valve"
@export var description : String
@export var suffix : String
var interacting = false

func _physics_process(_delta: float) -> void:
	if label != null:
		if interacting == false:
			label.hide()
		else:
			label.show()
			interacting = false
	
	update_rotation()
	

func interact(primary_interact : bool = false, secondary_interact : bool = false) -> void:
	if primary_interact == true:
			valve_position += step
	elif secondary_interact == true:
		valve_position -= step
	
	valve_position = clamp(valve_position, valve_min, valve_max)
	
	if label != null:
		interacting = true
		display_text(round_place(valve_position * 100,0), " " + suffix)


func update_rotation():
	if type == "valve":
		var start_rotation = 22.6
		get_parent().rotation.x = lerpf(get_parent().rotation.x, deg_to_rad(start_rotation - angle_range + abs(valve_position - 1) * (angle_range*2)), 0.2)
	elif type == "lever":
		var start_rotation = 0
		get_parent().rotation.z = lerpf(get_parent().rotation.z, deg_to_rad(start_rotation + angle_range - abs(valve_position - 1) * (angle_range*2)), 0.2)



func display_text(value, suffix_):
	label.text = description + "\n" + str(value, suffix_)

func round_place(num,places) -> float:
	return (round(num*pow(10,places))/pow(10,places))
