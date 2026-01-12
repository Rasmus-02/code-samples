extends Node
class_name AnimationHandler

var agent: Agent
var base_speed: float

func _ready() -> void:
	agent = get_parent()
	base_speed = agent.speed

func play_animation(anim_name: String, speed_scale: float = 1.0) -> void:
	var animation_zoom : int
	match Settings.graphics_option:
		1: # Potato
			animation_zoom = 0
		1: # Low
			animation_zoom = Globals.max_camera_zoom
		2: # Medium
			animation_zoom = Globals.min_camera_zoom * 0.5
		3: # High
			animation_zoom = Globals.min_camera_zoom
		
	if Globals.camera_zoom < animation_zoom or (anim_name != "Walk" and anim_name != "Idle"):
		if agent.animation_player.current_animation != anim_name and agent.has_animations and (agent.despawn_flag == false or (anim_name == "Infect" or anim_name == "Die")):
			if agent.animation_player.has_animation(anim_name):
				if anim_name == "Walk":# :)
					agent.animation_player.speed_scale = agent.speed / base_speed
				else:
					agent.animation_player.speed_scale = speed_scale
				agent.animation_player.play(anim_name)
			else:
				printerr(agent.name + " has no animation called " + anim_name + ".")
		elif agent.debug:
			printerr(agent.name + " has no animations.")
	else:
		if agent.animation_player.is_playing():
			agent.animation_player.stop()

func stop_animation() -> void:
	if agent.despawn_flag == false:
		agent.animation_player.stop()

func set_animation_speed(value) -> void:
	agent.animation_player.speed_scale = value
