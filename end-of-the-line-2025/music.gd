extends Node3D

@onready var intense : AudioStreamPlayer = $Intense
@onready var forrest : AudioStreamPlayer = $Forrest
@onready var buy : AudioStreamPlayer = $Buy
@onready var sell : AudioStreamPlayer = $Sell


func fade_in(music : AudioStreamPlayer):
	music.play()
	var tween = create_tween()
	tween.tween_property(music, "volume_db", -5, 5)

func fade_out(music : AudioStreamPlayer):
	music.stop()
	var tween = create_tween()
	tween.tween_property(music, "volume_db", -40, 5)

func play_sound_effect(sound_effect : AudioStreamPlayer):
	sound_effect.play()
