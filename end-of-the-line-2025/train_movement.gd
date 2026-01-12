extends Node3D

@export var drag = 0.02 # Air and ground friction
@export var max_pressure : int = 10
@export var brake_force : int = 20
@export var track : PathFollow3D
@export var engine : Node3D
@export var steam_release : Area3D
@export var throttle_lever : Area3D
@export var brake_lever : Area3D
@onready var pressure_gauge = $"Model/Steam Gauge/Pointer"
@onready var speedo_gauge = $Model/Speedo/Pointer
@onready var train_move_sound: AudioStreamPlayer3D = $TrainMove
@onready var steam_sound: AudioStreamPlayer3D = $"Steam"

var speed : float = 10 # Kmh
var pressure : float = 3
var acceleration : float
var exploded: bool

@onready var wheel_1: MeshInstance3D = $Model/Wheel1
@onready var wheel_2: MeshInstance3D = $Model/Wheel2
@onready var wheel_3: MeshInstance3D = $Model/Wheel3
@onready var wheel_4: MeshInstance3D = $Model/Wheel4
@onready var wheel_small_1: MeshInstance3D = $"Model/Wheel Small 1"
@onready var wheel_small_2: MeshInstance3D = $"Model/Wheel small 2"

@onready var upgrades: Node = $Upgrades
@onready var animation_player: AnimationPlayer = $AnimationPlayer

func _physics_process(delta: float) -> void:
	rotate_wheels()
	#Update values from valves and levers
	var steam_valve = steam_release.valve_position

	if speed < upgrades.min_speed and pressure < 1:
		steam_valve = 0
	var throttle = throttle_lever.valve_position
	var brake = brake_lever.valve_position
	
	#Calculate pressure in the steam engine
	pressure += (engine.power * 0.001)
	if pressure > max_pressure * upgrades.pressure_modifier and !exploded:
		exploded = true
		Globals.game_over.emit(false)
	
	#Calculate acceleration from pressure
	if speed <= throttle * 100:
		acceleration = pressure * 5
	else:
		acceleration = 0
	
	#Reduce pressure when accelerating
	pressure -= acceleration * 0.0001
	pressure -= (steam_valve ** 2) * 0.03
	pressure = clamp(pressure, 0, 100)
	
	#Calculate drag and brakes
	acceleration -= drag * (speed ** 2)
	if speed > upgrades.min_speed:
		acceleration -= brake * brake_force
	
	
	#Apply acceleration and drag
	speed += acceleration * delta
	
	speed = clamp(speed, 0, 101)
	
	#Move the train
	track.progress += speed * 0.2777 * delta
	
	update_dials()
	
	#sound
	train_move_sound.pitch_scale = speed * 0.05
	steam_sound.pitch_scale = pressure * 0.1
	
func update_dials():
	speedo_gauge.rotation.y = deg_to_rad(140.0 - (280.0 / 100.0 * speed))
	pressure_gauge.rotation.y = deg_to_rad(140.0 - (280.0 / max_pressure * pressure))

func round_place(num,places) -> float:
	return (round(num*pow(10,places))/pow(10,places))

func rotate_wheels() -> void:
	animation_player.speed_scale = speed * 0.2777 / 4.14
