extends CharacterBody2D

#region Variables
signal crash

@export var player = true

var wheelbase = 1000
var max_steering_angle = 15
var steering_angle = max_steering_angle
var max_engine_power = 1
var engine_power = max_engine_power
var ground_friction = -0.02
var drag = -0.00005
var drag_rpm_limit = -0.00001
var brake_force = -1000
var max_braking_acceleration : Vector2
var max_drag_acceleration : Vector2
var max_speed_reverse = 700
var slip_speed = 200
var traction_fast_max = 0.12
var traction_fast = traction_fast_max
var traction_slow = 0.1
var max_tire_limit = 1200.0
var tire_grip = 1200.0
var tire_limit = max_tire_limit
var drive_train = [1.0, 1.0]
var speed_kmh = 1
var engine_rpm = 1
var max_engine_rpm = 1
var drive_train_type = 0 #0=RWD, 1=FWD, 2=AWD
var drivetrain_loss = 1.0
var weight = 1
var treadwear = 1
var downforce = 1.0
var max_driveshaft_hp = 1
var max_driveshaft_tq = 1
var engine_cool_mod = 1.0
var brake_cooling = 1.0 #Celcius per second (road: 1-3, race: 2-5)
var brake_fade_limit = 1.0 #Degrees celcius (250-300 normal)
var handling_bonus = 1.0
var slide_ammount = 1.0
var engine_bay_size = [1,1]
var gear_ratio = null
var max_rpm = 1.0
var max_gear = 1
var gear = 1
var shift_cooldown = false
var engine_pitch = 1.0
var engine_volume_dampening = 0.0
var exhaust_tq_mod = 1.0
@onready var shift_timer = null

var acceleration = Vector2.ZERO
var d = 0 #direction the car is travelling in
var steer_direction
signal export_variables(speed, gear_ratio, gear, shift_cooldown, engine_position, drive_train_type, engine_pitch, engine_volume_dampening, grip, car_cooling, f, b, l , r, c)
@onready var engine = $Engine
@onready var player_inputs = $Player_Inputs
@onready var ai_inputs = $AI
@onready var effects = $Effects
var forward : float
var backward : float
var left : float
var right : float
var clutch : float
var handbrake : float
var rubberbanding = 1.0
var on_road = true
var last_on_road_position = Vector2.ZERO

@export var selected_car_key = 0
@export var selected_engine = 0
@export var selected_car = 0
@export var selected_chassi = 0
@export var selected_driveshaft = 0
@export var selected_subframe = 0
@export var selected_fenders = 0
@export var selected_f_bumper = 0
@export var selected_r_bumper = 0
@export var selected_hood = 0
@export var selected_headlights = 0
@export var selected_taillights = 0
@export var selected_spoiler = 0
@export var selected_mirrors = 0
@export var selected_brakes = 0
@export var selected_suspension = 0
@export var selected_tires = 0
@export var selected_wheels = 0
@export var selected_gearbox = 0
@export var selected_radiator = 0
@export var selected_exhaust = 0
@export var selected_color : Color
@export var in_garage = 0 #the key for the car the engine is in, if null engine is equippable in other cars
var engine_position = Vector2(0,0)
var engine_position_offset = Vector2(0,0)
var spawn_override_key = null
var fuel_type : String = "Gasoline"

var car_durability = 100
var chassi_durability = 100
var driveshaft_durability = 100
var subframe_durability = 100
var fenders_durability = 100
var f_bumper_durability = 100
var r_bumper_durability = 100
var hood_durability = 100
var headlights_durability = 100
var taillights_durability = 100
var spoiler_durability = 100
var mirrors_durability = 100
var brakes_durability = 100
var suspension_durability = 100
var tires_durability = 100
var wheels_durability = 100
var gearbox_durability = 100
var radiator_durability = 100
var exhaust_durability = 100

@export var chassi_color = Color(1,1,1,1)
@export var fenders_color = Color(1,1,1,1)
@export var f_bumper_color = Color(1,1,1,1)
@export var r_bumper_color = Color(1,1,1,1)
@export var hood_color = Color(1,1,1,1)
@export var headlights_color = Color(1,1,1,1)
@export var taillights_color = Color(1,1,1,1)
@export var spoiler_color = Color(1,1,1,1)
@export var mirrors_color = Color(1,1,1,1)

var specific_parts = null
var universal_parts = null
var chassi = null
var driveshaft = null
var subframe = null
var fenders = null
var f_bumper = null
var r_bumper = null
var hood = null
var headlights = null
var taillights = null
var spoiler = null
var mirrors = null
var brakes = null
var suspension = null
var tires = null
var tire_list = []
var wheel_list = []
var wheels = null
var gearbox = null
var exhaust = null
var radiator = null
var parts = [chassi, driveshaft, subframe, fenders, f_bumper, r_bumper, hood, headlights, taillights, spoiler, mirrors, gearbox, radiator,exhaust,brakes,suspension,tires,wheels]

var tiresmoke = preload("res://Assets/Effects/TireSmoke.tscn")
var tiremark = preload("res://Assets/Effects/tiremark.tscn")
var spawn_rotation = 0
#endregion

#region Import Export

#IMPORT STATS FROM ENGINE
func _on_engine_rpm_info(rpm, import_max_rpm):
	engine_rpm = rpm
	max_engine_rpm = import_max_rpm

func _on_engine_stats(_horsepower, torque, _max_torque, top_end_fuel_type):
	max_engine_power = (torque) * (1-drivetrain_loss) * exhaust_tq_mod
	engine_power = (max_engine_power * 150) * rubberbanding
	fuel_type = top_end_fuel_type

#EXPORT STATS FROM CAR
func export_signal():
	emit_signal("export_variables", speed_kmh, gear_ratio, gear, shift_cooldown, engine_position, drive_train_type, engine_pitch, engine_volume_dampening, tire_limit / max_tire_limit, engine_cool_mod, forward, backward, left, right, clutch)

func gearbox_function():
	if loaded == true and SelectedScene.scene == "Track":
		#gear ratio index 0 = final drive
		if (gear_ratio[gear - 1] * gear_ratio[0] * speed_kmh) * 10 >= max_engine_rpm and shift_cooldown == false and speed_kmh > 15 and gear < max_gear:
			gear += 1
			shift_cooldown = true
			shift_timer.start()
		elif ((gear_ratio[gear - 2] * gear_ratio[0] * speed_kmh) * 14) < max_engine_rpm * 1 and shift_cooldown == false and gear != 2:
			gear -= 1
			shift_cooldown = true
			shift_timer.start()
	elif running_dyno == 1:
		if shift_cooldown == false and gear < 3:
			if (gear_ratio[gear] * gear_ratio[0] * speed_kmh) * 10 >= 1500: #next gear rpm
				gear += 1
				shift_cooldown = true
				shift_timer.start()
		elif gear >= 3 and engine_rpm >= max_engine_rpm:
			running_dyno = 2
			engine.dyno = 3
	gear = clamp(gear, 2, max_gear)
	

func _on_gear_change_timer_timeout():
	if shift_timer.time_left <= 0:
		if loaded == true:
			shift_cooldown = false

var deleted = false
func delete_car():
	deleted = true
	#Clears the instanses in the car
	parts = [chassi, driveshaft, subframe, fenders, f_bumper, r_bumper, hood, headlights, taillights, spoiler, mirrors, gearbox, radiator,exhaust,brakes,suspension,tires,wheels]
	for i in parts.size():
		if parts[i] != null:
			parts[i].free()
	parts.clear()
	if tiresmoke_instanse != null:
		tiresmoke_instanse.free()
	#Go into part selector and clear
	var part_selector = get_child(0).get_child(0).get_child(0)
	part_selector.reload_car(true)
	engine.delete_engine()
	specific_parts.queue_free()
	universal_parts.queue_free()
	self.queue_free()

func _init():
	Utils.connect("changing_scene", Callable(self, "_scene_changed"))
func _scene_changed():
	if player == true:
		Save_Load.edit_car(self)

func load_car(index):
	var load_file = Save_Load.select_car(index)
	if load_file != null:
		#Loads engine from JSON
		selected_car = load_file.car_id
		selected_car_key = load_file.key
		selected_engine = load_file.engine
		if engine != null:
			engine.load_engine(selected_engine)
		Save_Load.temp_key = selected_engine
		selected_chassi = load_file.chassi[0]
		selected_f_bumper = load_file.f_bumper[0]
		selected_r_bumper = load_file.r_bumper[0]
		selected_fenders = load_file.fenders[0]
		selected_hood = load_file.hood[0]
		selected_mirrors = load_file.mirrors[0]
		selected_headlights = load_file.headlights[0]
		selected_taillights = load_file.taillights[0]
		selected_spoiler = load_file.spoiler[0]
		selected_suspension = load_file.suspension[0]
		selected_wheels = load_file.wheels[0]
		selected_tires = load_file.tires[0]
		selected_brakes = load_file.brakes[0]
		selected_subframe = load_file.subframe[0]
		selected_driveshaft = load_file.driveshaft[0]
		selected_gearbox = load_file.gearbox[0]
		selected_radiator = load_file.radiator[0]
		selected_exhaust = load_file.exhaust[0]
		in_garage = load_file.in_garage
		gear_ratio = load_file.gear_ratio
		#Color got a bit complicated because JSON file made color into a string so now we convert it into r g and b values
		var c = load_file.color #color string
		var r = ""
		var g = ""
		var b = ""
		var rgb_counter = 0
		var i = 0
		#Algorithm for converting the color string to r, g, b
		while rgb_counter < 3:
			var string = str(c)[i]
			if string != "(" and string != ")" and string != "," and string != " ":
				match rgb_counter:
					0:
						r += string
					1:
						g += string
					2:
						b += string
			if string == ",":
				rgb_counter += 1
			i+=1
		selected_color = Color(float(r),float(g),float(b),1.0) #sets the color with alpha of 1
		
		chassi_color = load_file.chassi[2]
		fenders_color = load_file.fenders[2]
		f_bumper_color = load_file.f_bumper[2]
		r_bumper_color = load_file.r_bumper[2]
		hood_color = load_file.hood[2]
		headlights_color = load_file.headlights[2]
		taillights_color = load_file.taillights[2]
		spoiler_color = load_file.spoiler[2]
		mirrors_color = load_file.mirrors[2]
		
		#Damage stats
		chassi_durability = load_file.chassi[1]
		f_bumper_durability = load_file.f_bumper[1]
		r_bumper_durability = load_file.r_bumper[1]
		fenders_durability = load_file.fenders[1]
		hood_durability = load_file.hood[1]
		mirrors_durability = load_file.mirrors[1]
		headlights_durability = load_file.headlights[1]
		taillights_durability = load_file.taillights[1]
		spoiler_durability = load_file.spoiler[1]
		suspension_durability = load_file.suspension[1]
		wheels_durability = load_file.wheels[1]
		tires_durability = load_file.tires[1]
		brakes_durability = load_file.brakes[1]
		subframe_durability = load_file.subframe[1]
		driveshaft_durability = load_file.driveshaft[1]
		gearbox_durability = load_file.gearbox[1]
		radiator_durability = load_file.radiator[1]
		exhaust_durability = load_file.exhaust[1]
	
	else:
		selected_car = 0
		selected_car_key = 0
		selected_engine = 0
		selected_chassi = 0
		selected_f_bumper = 0
		selected_r_bumper = 0
		selected_fenders = 0
		selected_hood = 0
		selected_mirrors = 0
		selected_headlights = 0
		selected_taillights = 0
		selected_spoiler = 0
		selected_suspension = 0
		selected_wheels = 0
		selected_tires = 0
		selected_brakes = 0
		selected_subframe = 0
		selected_driveshaft = 0
		selected_gearbox = 0
		selected_radiator = 0
		selected_exhaust = 0
		selected_color = Color(0.5,0.5,0.5, 1)
		chassi_color = 0
		fenders_color = 0
		f_bumper_color = 0
		r_bumper_color = 0
		hood_color = 0
		headlights_color = 0
		taillights_color = 0
		spoiler_color = 0
		mirrors_color = 0
		
		#Damage stats
		chassi_durability = 100
		f_bumper_durability = 100
		r_bumper_durability = 100
		fenders_durability = 100
		hood_durability = 100
		mirrors_durability = 100
		headlights_durability = 100
		taillights_durability = 100
		spoiler_durability = 100
		suspension_durability = 100
		wheels_durability = 100
		tires_durability = 100
		brakes_durability = 100
		subframe_durability = 100
		driveshaft_durability = 100
		gearbox_durability = 100
		radiator_durability = 100
		exhaust_durability = 100
	get_node("Car_spawner").reload_car()
	update_car_parts()

func load_car_from_algorithm(dictionary):
	selected_car = dictionary.Car_ID
	selected_car_key = 9999
	selected_engine = 9999
	selected_chassi = 0
	selected_f_bumper = dictionary.f_bumper
	selected_r_bumper = dictionary.r_bumper
	selected_fenders = dictionary.fenders
	selected_hood = dictionary.hood
	selected_mirrors = dictionary.mirrors
	selected_headlights = dictionary.headlight
	selected_taillights = dictionary.taillights
	selected_spoiler = dictionary.spoiler
	selected_suspension = dictionary.suspension
	selected_wheels = dictionary.wheels
	selected_tires = dictionary.tires
	selected_brakes = dictionary.brakes
	selected_subframe = dictionary.subframe
	selected_driveshaft = dictionary.driveshaft
	selected_gearbox = dictionary.gearbox
	selected_radiator = dictionary.radiator
	selected_exhaust = dictionary.exhaust
	
	chassi_color = 0
	fenders_color = 0
	f_bumper_color = 0
	r_bumper_color = 0
	hood_color = 0
	headlights_color = 0
	taillights_color = 0
	spoiler_color = 0
	mirrors_color = 0
	
	#Damage stats
	if dictionary.get("durability") != null:
		chassi_durability = dictionary.durability.get("chassi")
		f_bumper_durability = dictionary.durability.get("f_bumper")
		r_bumper_durability = dictionary.durability.get("r_bumper")
		fenders_durability = dictionary.durability.get("fenders")
		hood_durability = dictionary.durability.get("hood")
		mirrors_durability = dictionary.durability.get("mirrors")
		headlights_durability = dictionary.durability.get("headlights")
		taillights_durability = dictionary.durability.get("taillights")
		spoiler_durability = dictionary.durability.get("spoiler")
		suspension_durability = dictionary.durability.get("suspension")
		wheels_durability = dictionary.durability.get("wheels")
		tires_durability = dictionary.durability.get("tires")
		brakes_durability = dictionary.durability.get("brakes")
		subframe_durability = dictionary.durability.get("subframe")
		driveshaft_durability = dictionary.durability.get("driveshaft")
		gearbox_durability = dictionary.durability.get("gearbox")
		radiator_durability = dictionary.durability.get("radiator")
		exhaust_durability = dictionary.durability.get("exhaust")
	else:
		chassi_durability = 100
		f_bumper_durability = 100
		r_bumper_durability = 100
		fenders_durability = 100
		hood_durability = 100
		mirrors_durability = 100
		headlights_durability = 100
		taillights_durability = 100
		spoiler_durability = 100
		suspension_durability = 100
		wheels_durability = 100
		tires_durability = 100
		brakes_durability = 100
		subframe_durability = 100
		driveshaft_durability = 100
		gearbox_durability = 100
		radiator_durability = 100
		exhaust_durability = 100
	get_node("Car_spawner").reload_car()
	update_car_parts()

# Remove all hitboxes from car, the parts will automatically add a new correct hitbox
func reload_hitboxes():
	for child in get_children():
		if child is CollisionPolygon2D or child is CollisionShape2D:
			child.queue_free()

func update_car_parts(): 
	reload_hitboxes()
	#selects the car
	self.get_child(0).current_car = self.get_child(0).car_list[selected_car]
	#sends signal to part list to update car
	self.get_child(0).get_child(0).get_child(0).update = true
	#sends the updated parts to part list (part selector)
	self.get_child(0).get_child(0).get_child(0).selected_chassi = selected_chassi
	self.get_child(0).get_child(0).get_child(0).selected_driveshaft = selected_driveshaft
	self.get_child(0).get_child(0).get_child(0).selected_subframe = selected_subframe
	self.get_child(0).get_child(0).get_child(0).selected_fenders = selected_fenders
	self.get_child(0).get_child(0).get_child(0).selected_f_bumper = selected_f_bumper
	self.get_child(0).get_child(0).get_child(0).selected_r_bumper = selected_r_bumper
	self.get_child(0).get_child(0).get_child(0).selected_hood = selected_hood
	self.get_child(0).get_child(0).get_child(0).selected_headlights = selected_headlights
	self.get_child(0).get_child(0).get_child(0).selected_taillights = selected_taillights
	self.get_child(0).get_child(0).get_child(0).selected_spoiler = selected_spoiler
	self.get_child(0).get_child(0).get_child(0).selected_mirrors = selected_mirrors
	self.get_child(0).get_child(0).get_child(0).selected_brakes = selected_brakes
	self.get_child(0).get_child(0).get_child(0).selected_suspension = selected_suspension
	self.get_child(0).get_child(0).get_child(0).selected_tires = selected_tires
	tire_list = self.get_child(0).get_child(0).get_child(0).tire_list
	wheel_list = self.get_child(0).get_child(0).get_child(0).wheel_list
	self.get_child(0).get_child(0).get_child(0).selected_wheels = selected_wheels
	self.get_child(0).get_child(0).get_child(0).selected_gearbox = selected_gearbox
	self.get_child(0).get_child(0).get_child(0).selected_radiator = selected_radiator
	self.get_child(0).get_child(0).get_child(0).selected_exhaust = selected_exhaust
	self.get_child(0).get_child(0).get_child(0).in_garage = in_garage
	update_stats()

#updates the stats from car_spawner
func update_stats():
	specific_parts = get_child(0).get_child(0).get_child(0).get_child(0) #from part list specific to the engine
	universal_parts = get_child(0).get_child(0).get_child(0).get_child(1) #from universal part list
	var part_selector = get_child(0).get_child(0).get_child(0)
	
	#Clears the list
	parts = [chassi, driveshaft, subframe, fenders, f_bumper, r_bumper, hood, headlights, taillights, spoiler, mirrors, gearbox, radiator,exhaust,brakes,suspension,tires,wheels]
	for i in parts.size():
		if parts[i] != null:
			parts[i].queue_free()
	parts.clear()
	
	chassi = specific_parts.chassi[selected_chassi].instantiate()
	driveshaft = specific_parts.driveshaft[selected_driveshaft].instantiate()
	subframe = specific_parts.subframe[selected_subframe].instantiate()
	fenders = specific_parts.fenders[selected_fenders].instantiate()
	f_bumper = specific_parts.f_bumper[selected_f_bumper].instantiate()
	r_bumper = specific_parts.r_bumper[selected_r_bumper].instantiate()
	hood = specific_parts.hood[selected_hood].instantiate()
	headlights = specific_parts.headlights[selected_headlights].instantiate()
	taillights = specific_parts.taillights[selected_taillights].instantiate()
	spoiler = specific_parts.spoiler[selected_spoiler].instantiate()
	mirrors = specific_parts.mirrors[selected_mirrors].instantiate()
	gearbox = universal_parts.gearbox[selected_gearbox].instantiate()
	radiator = universal_parts.radiator[selected_radiator].instantiate()
	exhaust = specific_parts.exhaust[selected_exhaust].instantiate()
	brakes = universal_parts.brakes[selected_brakes].instantiate()
	suspension = universal_parts.suspension[selected_suspension].instantiate()
	tires = universal_parts.tires[selected_tires].instantiate()
	wheels = universal_parts.wheels[selected_wheels].instantiate()
	
	chassi.color = chassi_color
	fenders.color = fenders_color
	f_bumper.color = f_bumper_color
	r_bumper.color = r_bumper_color
	hood.color = hood_color
	headlights.color = headlights_color
	taillights.color = taillights_color
	spoiler.color = spoiler_color
	mirrors.color = mirrors_color
	
	# Stats not affected by damage
	weight = chassi.weight + driveshaft.weight + subframe.weight + fenders.weight + f_bumper.weight + r_bumper.weight + hood.weight + headlights.weight + taillights.weight + spoiler.weight + mirrors.weight + brakes.weight + suspension.weight + tires.weight + wheels.weight  + gearbox.weight + radiator.weight + exhaust.weight + engine.weight
	treadwear = tires.treadwear
	drive_train_type = driveshaft.drivetrain
	brake_cooling = brakes.brake_cooling + wheels.brake_cooling
	brake_fade_limit = brakes.brake_fade_limit
	max_driveshaft_tq = driveshaft.max_torque
	wheelbase = chassi.wheelbase
	engine_bay_size = [chassi.engine_bay_lenght - radiator.width, chassi.engine_bay_width]
	max_gear = gearbox.gear_ratio.size()
	shift_timer = get_child(0).get_child(0).get_node("Gear_Change_Timer")#node.shift_timer
	engine_position = part_selector.engine_position
	#Engine position offset used in size calculations
	if drive_train_type == 1: #FWD 
		engine_position_offset = Vector2(gearbox.get_node("Engine").position.y + driveshaft.get_node("Gearbox").position.x - chassi.engine_bay_start_lenght, driveshaft.get_node("Gearbox").position.y - gearbox.get_node("Engine").position.x)
	else: #RWD AWD
		engine_position_offset = Vector2(gearbox.get_node("Engine").position.x + driveshaft.get_node("Gearbox").position.x - chassi.engine_bay_start_lenght, driveshaft.get_node("Gearbox").position.y - gearbox.get_node("Engine").position.y)
	engine_pitch = exhaust.pitch_tweak
	engine_volume_dampening = exhaust.sound_dampening
	
	# Stats affected by damage
	damage_calculator_collision("None", 0) # Update the damage stats when loading in
	
	is_ready = true
	
	#If Saved gear ratio is missing gears, reset it to default
	if gear_ratio == null or gear_ratio.size() < gearbox.gear_ratio.size():
		gear_ratio = gearbox.gear_ratio

func update_durability():
	chassi.durability = chassi_durability
	f_bumper.durability = f_bumper_durability
	r_bumper.durability = r_bumper_durability
	fenders.durability = fenders_durability
	hood.durability = hood_durability
	mirrors.durability = mirrors_durability
	headlights.durability = headlights_durability
	taillights.durability = taillights_durability
	spoiler.durability = spoiler_durability
	suspension.durability = suspension_durability
	wheels.durability = wheels_durability
	tires.durability = tires_durability
	brakes.durability = brakes_durability
	subframe.durability = subframe_durability
	driveshaft.durability = driveshaft_durability
	gearbox.durability = gearbox_durability
	radiator.durability = radiator_durability
	exhaust.durability = exhaust_durability

func is_functional():
	if selected_driveshaft != 0 and selected_subframe != 0 and selected_suspension != 0 and selected_tires != 0 and selected_wheels != 0 and selected_gearbox != 0 and selected_radiator != 0:
		return(true)
		
	else:
		return(false)

func set_spawn_rotation():
	#Rotate the car according to how it should be spawned in garage or scrapyard
	if spawn_rotation > 0 and engine.position_loaded == true:
			self.rotation = deg_to_rad(spawn_rotation)
			spawn_rotation = 0

var is_ready = false
func car_constructor():
	if player == true:
		if spawn_override_key == null:
			load_car(Save_Load.selected_car_key)
		else:
			load_car(spawn_override_key)
	traction_constructor() #TEST Needs to be changed location

var loaded = false
func is_loaded():
	#For making the car function on track
	if tire_list.size() != 0 and tire_list[3] != null and (SelectedScene.scene == "Track" or running_dyno != 0):
		loaded = true
		return true
	elif engine != null:
		engine.is_running = false

func _physics_process(delta):
	set_spawn_rotation()
	export_signal()
	if is_ready == false:
			car_constructor()
	if is_functional() and deleted == false:
		if is_loaded() and chassi != null:
			speed_kmh = velocity.length()*.02*3.6
			tire_limit = clamp(tire_limit, 0, max_tire_limit) #clamps the tirelimit
			acceleration = Vector2.ZERO
			light_controll("headlights",true, 1)
			gearbox_function()
			_on_gear_change_timer_timeout()
			if SelectedScene.scene == "Track" and RaceStatus.started == true: #Make it so the car can move, on dyno the car should not move
				rubberbanding = $Placing.performance_bonus
				if RaceStatus.finished == true and player == true:
					player = false
				get_input()
				calculate_steering(delta)
				move_and_slide()
				wheel_controller()
				traction_controller()
				sliding()
				collision_handler()
				damage_calculator_tick()
			if running_dyno == 1:
				Utils.blocked = true
				engine.is_running = true
				dyno_controller()
			elif running_dyno == 2:
				Utils.blocked = false
				end_dyno_run()
			elif running_dyno == 0:
				engine.is_running = true #Turns on the engine
			apply_friction()
			velocity += acceleration * delta
			var sound_rng = randf_range(-0.05, 0.05)
			#Tire sound
			$"Sound effects/Tire Screetch".volume_db = (-84.0 + (80.0 - 80.0 * (float(tire_limit) / float(max_tire_limit) +.01)**1.5))
			$"Sound effects/Tire Screetch".pitch_scale = 1 + (sound_rng*1.2 * (1 + drag*-10000))
			
			#Wind Sound
			if player == true:
				$"Sound effects/Wind Noise".volume_db = (-80 + 80 * ((speed_kmh / 200)** 0.2))
				$"Sound effects/Wind Noise".pitch_scale = 1 + 1 * (speed_kmh / 500)
			else:
				$"Sound effects/Wind Noise".volume_db = -80
		else:
			$"Sound effects/Tire Screetch".volume_db = -80
			$"Sound effects/Tire Screetch".pitch_scale = 1
			$"Sound effects/Wind Noise".volume_db = -80
			$"Sound effects/Wind Noise".pitch_scale = 1
			velocity = Vector2.ZERO


#region Tiresmoke
var spawn_emitters = true
var tiresmoke_instanse = null
var tiremark_instanse = null
var tire_smoke_array = [null,null,null,null]
var tiremark_array = [null,null,null,null]
var smoke_amount = 0
var wheels_on_road = 4
func wheel_controller():
	if selected_wheels != 0:
		var state = false
		var FL = tire_list[0]
		var FR = tire_list[3]
		var RL = tire_list[1]
		var RR = tire_list[2]
		var FL_Wheel = wheel_list[0]
		var FR_Wheel = wheel_list[3]
		var RL_Wheel = wheel_list[1]
		var RR_Wheel = wheel_list[2]
		var wheel_array = [FL,FR,RL,RR]
		FR.rotation = deg_to_rad(turn*steering_angle*2)
		FL.rotation = deg_to_rad(turn*steering_angle*2)
		FR_Wheel.rotation = deg_to_rad(turn*steering_angle*2)
		FL_Wheel.rotation = deg_to_rad(turn*steering_angle*2)
		if spawn_emitters == true:
			for i in 4:
				tiresmoke_instanse = tiresmoke.instantiate() #Indexes [FL,FR,RL,RR]
				tiremark_instanse = tiremark.instantiate()  #Indexes [FL,FR,RL,RR]
				
				tiresmoke_instanse.get_child(0).set_emitting(false)
				tire_smoke_array[i] = tiresmoke_instanse
				tiremark_array[i] = tiremark_instanse
				tiresmoke_instanse.position = wheel_array[i].position
				tiremark_instanse.position = wheel_array[i].position
				self.add_child(tiresmoke_instanse)
				self.add_child(tiremark_instanse)
			spawn_emitters = false
		
		var n = 0
		var wheels_on_road_array = []
		wheels_on_road_array.clear()
		if SelectedScene.scene != "garage":
			for i in 4:
				if wheel_array[i].get_node("Hitbox").on_road:
					n += 1
					wheels_on_road_array.append(true)
				else:
					wheels_on_road_array.append(false)
			wheels_on_road = n
			
			#When going off the road completely, save position to teleport to if out off bounds
			if wheels_on_road == 0:
				if on_road == true:
					last_on_road_position = self.global_position
				on_road = false
			#If back on road, reset on_road status
			else:
				on_road = true
		else:
			wheels_on_road_array = [true, true, true, true]
		
		max_tire_limit = (tire_grip * 0.5 * (wheels_on_road * 0.25) + (tire_grip * 0.5)) * rubberbanding * $Placing.grip_bonus
		
		#Increases the smoke density when losing grip
		smoke_amount = 1-(tire_limit/(max_tire_limit*0.6))
		if smoke_amount > 0.1:
			state = true
		else:
			state = false
		#Smoke for front wheels
		if (drive_train[1]-1)/1 > (drive_train[0]-1)/0.4:
			if wheels_on_road_array[0] == true:
				tire_smoke_array[0].get_child(0).set_emitting(state)
			if wheels_on_road_array[1] == true:
				tire_smoke_array[1].get_child(0).set_emitting(state)
			tire_smoke_array[2].get_child(0).set_emitting(false)
			tire_smoke_array[3].get_child(0).set_emitting(false)
			
			if smoke_amount > 0.6:
				if wheels_on_road_array[0] == true:
					tiremark_array[0].draw_tiremark(true, FL.global_position)
				else:
					tiremark_array[0].draw_tiremark(false, FL.global_position)
				if wheels_on_road_array[1] == true:
					tiremark_array[1].draw_tiremark(true, FR.global_position)
				else:
					tiremark_array[1].draw_tiremark(false, FR.global_position)
				tiremark_array[2].draw_tiremark(false, RL.global_position)
				tiremark_array[3].draw_tiremark(false, RR.global_position)
			else:
				tiremark_array[0].draw_tiremark(false, FL.global_position)
				tiremark_array[1].draw_tiremark(false, FR.global_position)
				tiremark_array[2].draw_tiremark(false, RL.global_position)
				tiremark_array[3].draw_tiremark(false, RR.global_position)
			# Smoke particle
			for smoke in tire_smoke_array:
				smoke.get_child(0).modulate = Color(1,1,1, smoke_amount - 0.4)
				smoke.get_child(0).process_material.scale_max = 6 * (smoke_amount - 0.4)
				smoke.get_child(0).process_material.scale_min = smoke.get_child(0).process_material.scale_max * 0.5
			
		#Smoke for rear wheels
		else:
			tire_smoke_array[0].get_child(0).set_emitting(false)
			tire_smoke_array[1].get_child(0).set_emitting(false)
			if wheels_on_road_array[2] == true:
				tire_smoke_array[2].get_child(0).set_emitting(state)
			if wheels_on_road_array[3] == true:
				tire_smoke_array[3].get_child(0).set_emitting(state)
			
			if smoke_amount > 0.6:
				tiremark_array[0].draw_tiremark(false, FL.global_position)
				tiremark_array[1].draw_tiremark(false, FR.global_position)
				if wheels_on_road_array[2] == true:
					tiremark_array[2].draw_tiremark(true, RL.global_position)
				else:
					tiremark_array[2].draw_tiremark(false, RL.global_position)
				if wheels_on_road_array[3] == true:
					tiremark_array[3].draw_tiremark(true, RR.global_position)
				else:
					tiremark_array[3].draw_tiremark(false, RR.global_position)
			else:
				tiremark_array[0].draw_tiremark(false, FL.global_position)
				tiremark_array[1].draw_tiremark(false, FR.global_position)
				tiremark_array[2].draw_tiremark(false, RL.global_position)
				tiremark_array[3].draw_tiremark(false, RR.global_position)
			# Smoke particle
			for smoke in tire_smoke_array:
				smoke.get_child(0).modulate = Color(1,1,1, smoke_amount - 0.4)
				smoke.get_child(0).process_material.scale_max = 6 * (smoke_amount - 0.4)
				smoke.get_child(0).process_material.scale_min = smoke.get_child(0).process_material.scale_max * 0.5
#endregion


func collision_handler(): #If car crashes into something or gets crashed into
	if get_slide_collision_count() != 0 and effects.get_node("Collision").emitting == false:
		var collision_index = get_slide_collision(get_slide_collision_count() - 1)
		var collision_velocity = abs((collision_index.get_collider_velocity() - get_real_velocity()).length()) #VELOCITY
		# Particle Effect
		var collision = effects.get_node("Collision")
		var collision_position = collision_index.get_position() #POSITION
		collision.z_index = self.z_index + 100
		collision.global_position = collision_position
		
		if collision_velocity > 500:
			collision.emitting = true
			$"Sound effects/AudioStreamPlayer2D".play()
			
			# Damage
		var collision_shape = collision_index.get_local_shape()
		damage_calculator_collision(collision_shape.name, collision_velocity)

# Called every tick
func damage_calculator_tick():
	if player == true: # no damage for AI
		
		# Driveshaft damage if above torque
		if engine.torque > driveshaft.max_torque:
			var damage = (engine.torque - driveshaft.max_torque) * 0.005
			driveshaft_durability -= damage
			driveshaft_durability = clamp(driveshaft_durability, 0, 100)
		
		# Gearbox damage if above torque
		if engine.torque > gearbox.max_tq:
			var damage = (engine.torque - gearbox.max_tq) * 0.005
			gearbox_durability -= damage
			gearbox_durability = clamp(gearbox_durability, 0, 100)
		
		# Tire damage when driving (extra damage if drifting)
		var slide_damage = (1 - (tire_limit / max_tire_limit)) * (treadwear * 0.02)
		var tire_wear = (treadwear / 60.0) * (float(speed_kmh) / 100.0) # treadwear is dps || More speed == more wear
		tires_durability -= (slide_damage + tire_wear) * 0.5
		tires_durability = clamp(tires_durability, 0, 100)

# Called during collision
func damage_calculator_collision(part : String, speed):
	emit_signal("crash")
	match part:
		"Chassi Hitbox":
			var damage = speed * 0.0001
			chassi_durability -= damage
			chassi_durability = clamp(chassi_durability, 0, 100)
		"Front Bumper Hitbox":
			var damage = speed * 0.0003
			f_bumper_durability -= damage
			f_bumper_durability = clamp(f_bumper_durability, 0, 100)
		"Rear Bumper Hitbox":
			var damage = speed * 0.0003
			r_bumper_durability -= damage
			r_bumper_durability = clamp(r_bumper_durability, 0, 100)
		"Fenders Hitbox":
			var damage = speed * 0.0003
			fenders_durability -= damage
			fenders_durability = clamp(fenders_durability, 0, 100)
		"Spoiler Hitbox":
			var damage = speed * 0.001
			spoiler_durability -= damage
			spoiler_durability = clamp(spoiler_durability, 0, 100)
	
	
	## Calculate new stats depending on durability
	
	# Drag
	var drag_mod = (durability_perfromance(chassi.drag, chassi_durability, "+") + durability_perfromance(hood.drag, hood_durability, "+") + 
	durability_perfromance(f_bumper.drag, f_bumper_durability, "+") + durability_perfromance(r_bumper.drag, r_bumper_durability, "+") + 
	durability_perfromance(fenders.drag, fenders_durability, "+") + durability_perfromance(spoiler.drag, spoiler_durability, "+") + 
	durability_perfromance(mirrors.drag, mirrors_durability, "+") + durability_perfromance(headlights.drag, headlights_durability, "+"))
	if selected_hood == 0:
		drag_mod += 0.08
	if selected_f_bumper == 0:
		drag_mod += 0.06
	if selected_r_bumper == 0:
		drag_mod += 0.04
	if selected_fenders == 0:
		drag_mod += 0.05
	if selected_headlights == 0:
		drag_mod += 0.04
	drag = -0.00008 * drag_mod
	
	# Downforce
	downforce = (durability_perfromance(chassi.downforce, chassi_durability, "-") + durability_perfromance(hood.downforce, hood_durability, "-") + 
	durability_perfromance(f_bumper.downforce, f_bumper_durability, "-") + durability_perfromance(r_bumper.downforce, r_bumper_durability, "-") + 
	durability_perfromance(fenders.downforce, fenders_durability, "-") + durability_perfromance(spoiler.downforce, spoiler_durability, "-") +
	durability_perfromance(mirrors.downforce, mirrors_durability, "-"))
	
	# Handling
	handling_bonus = ((1+((durability_perfromance(suspension.handling_bonus, suspension_durability, "-") + 
	durability_perfromance(subframe.handling_bonus, subframe_durability, "-")) * 0.5)/((1+(weight/1000.0))/2.0))/2)
	
	# Grip
	tire_grip = durability_perfromance(tires.grip, tires_durability, "-")
	
	# Brake force
	brake_force = (durability_perfromance(brakes.brake_force, brakes_durability, "-") / (weight/1000.0)) *-1
	
	# Power loss
	drivetrain_loss = durability_perfromance(driveshaft.drivetrain_loss, driveshaft_durability, "+")
	exhaust_tq_mod = (exhaust.tq_mod_exhaust + durability_perfromance(exhaust.tq_mod_exhaust, exhaust_durability, "-")) * 0.5
	
	# Shift time
	shift_timer.set_wait_time(durability_perfromance(gearbox.shift_time, gearbox_durability, "+"))
	
	# Cooling
	engine_cool_mod = (durability_perfromance(hood.engine_cool_mod, hood_durability, "-") + 
	durability_perfromance(f_bumper.engine_cool_mod, f_bumper_durability, "-") + durability_perfromance(radiator.cooling, radiator_durability, "-"))
	
	update_durability()


func durability_perfromance(stat, durability, type):
	# + / - == If stat increase or decrease when damaged
	match type:
		"+":
			return stat + (stat * 0.01  * (100 - durability))
		"-":
			return stat * ((float(durability + 1.0)  / 100)) ** 0.5



func out_of_bounds():
	global_position = last_on_road_position
	velocity = Vector2.ZERO



func light_controll(light_type,on_off, strenght):
	if selected_taillights != 0 and taillights != null and selected_headlights != 0 and headlights != null:
		var taillight_sprite = self.get_child(0).get_child(0).get_child(0).taillights.get_node("Sprite2D")
		if light_type == "rear": #Taillights
			var index = 0
			if on_off == true: #Set the light of the sprite
				taillight_sprite.modulate = Color(strenght, strenght, strenght)
				for i in 1: #Set the actual light coming of
					index = i
					var taillight = self.get_child(0).get_child(0).get_child(0).taillights.get_node("Taillights").get_child(index)
					taillight.visible = true
					taillight.energy = strenght / 2.0
					taillight.texture_scale = 0.1 * strenght
					i+=1
			elif on_off == false: #Set the light of the sprite
				taillight_sprite.modulate = Color(0,0,0)
				for i in 1: #Set the actual light coming of
					index = i
					var taillight = self.get_child(0).get_child(0).get_child(0).taillights.get_node("Taillights").get_child(index)
					taillight.visible = false
					taillight.energy = 0
					taillight.texture_scale = 0
					i+=1
		var headlight_strenght = strenght / 30.0
		var headlight_sprite = self.get_child(0).get_child(0).get_child(0).headlights.get_node("Sprite2D")
		if light_type == "headlights": #Headlights
			#Set the light of the sprite
			if on_off == true: #Turn on headlight sprite
				if headlight_sprite.vframes > 1: #If animated (popups)
					headlight_sprite.frame = 1
				headlight_sprite.get_child(0).show() #The unshaded lightmask
			else: #Turn off headlight sprite
				headlight_sprite.frame = 0
				headlight_sprite.get_child(0).show() #The unshaded lightmask
			var index = 0
			for i in 2: #Set the actual light coming of
				index = i
				var headlights_node = self.get_child(0).get_child(0).get_child(0).headlights.get_node("Lights").get_child(index)
				if i <= 1:
					headlights_node.energy = headlight_strenght * 2
				else:
					headlights_node.energy = headlight_strenght
				i+=1
		if light_type == "light_z": # change culling layers of headlights
			var height = 1
			if on_off == true:
				height = 3
			else:
				height = 1
			var index = 0
			for i in 1:
				index = i
				var taillights_node = self.get_child(0).get_child(0).get_child(0).taillights.get_node("Taillights").get_child(index)
				taillights_node.set_item_cull_mask(height) #Sets the lights to act on both layer 1 (track) and layer 2 (bridge)
				i+=1
			for n in 2:
				index = n
				var headlights_node = self.get_child(0).get_child(0).get_child(0).headlights.get_node("Lights").get_child(index)
				headlights_node.set_item_cull_mask(height)
				n+=1

var running_dyno = 0
func run_dyno():
	running_dyno = 1
	engine.dyno = 2
func dyno_controller():
	acceleration = (transform.x * ((engine_power / (weight / 10)) * ((1 + (tire_limit/max_tire_limit))/2)))
func end_dyno_run():
	if engine_rpm <= engine.idle_rpm:
		gear = 0
		engine.dyno = 0
		running_dyno = 0
		engine.is_running = false
		loaded = false


var turn = 0
func get_input():
	if player == true:
		forward = player_inputs.forward
		backward = player_inputs.backward
		left = player_inputs.left
		right = player_inputs.right
		clutch = player_inputs.clutch
		handbrake = player_inputs.handbrake
	else:
		forward = ai_inputs.forward
		backward = ai_inputs.backward
		left = ai_inputs.left
		right = ai_inputs.right
		clutch = 0
		handbrake = 0
	
	var turn_mod = 1.1 - (velocity.length() / 4500.0) ** 0.8
	turn_mod = clamp(turn_mod, 0.3, 1.1)
	
	#Right
	if right > 0:
		turn += right * handling_bonus * (0.02 * turn_mod)
	
	#Left
	if left > 0:
		turn -= left * handling_bonus * (0.02 * turn_mod)
	turn = clamp(turn, -turn_mod * left, turn_mod * right) #clamp to keep it at the selected turn
	
	
	#for resetting turn angle
	if left == 0 and right == 0:
		if turn != 0: #Resets the steering angle
			var turn_reset = 1-(handling_bonus/33) #Heavier cars take more time to go straight
			turn_reset = clamp(turn_reset, 0.9, 0.99)
			turn = turn*turn_reset
		if turn >= -0.015 and turn <= 0.015:
			turn = 0
	turn = clamp(turn, -1, 1)
	
	var tirelimit_scale = 0.2
	if abs(turn) > 0: #if turning (traction)
		tire_limit -= tirelimit_scale * ((weight * velocity.length() / 75000.0 * abs(turn)) / handling_bonus)
	#DRIFT BONUS (DISABLED) tire_limit -= tirelimit_scale * slide_ammount * 10
	tire_limit += tirelimit_scale * (max_tire_limit/120.0 + ((downforce * rubberbanding * speed_kmh) / 10000.0) + weight / 1000.0)
	steer_direction = turn * deg_to_rad(steering_angle)
	
	
	if forward > 0 and clutch <= 0:
		acceleration = forward * (transform.x * ((engine_power / (weight / 10)) * ((1 + (tire_limit/max_tire_limit))/2)))
		var drivetrain_grip_mod = 1
		if drive_train_type == 0: #RWD
			drivetrain_grip_mod = 1.05
		elif drive_train_type == 1: #FWD
			drivetrain_grip_mod = 0.85
		elif drive_train_type == 2: #AWD
			drivetrain_grip_mod = 1.3
		tire_limit -= (tirelimit_scale * (acceleration.length()/10/drivetrain_grip_mod)) * (tire_limit / max_tire_limit)
	max_braking_acceleration = (transform.x * (brake_force * 0.7)) #For The AI to use
	if backward > 0:
		if d > 0: #to check if car is moving forwards
			acceleration = backward * max_braking_acceleration * d
			tire_limit -= backward * (tirelimit_scale * (brake_force / -200.0))
			light_controll("rear",true, 1.7)
		else: #to check that car is not moving forwards
			acceleration = backward * (transform.x * ((-engine_power / (weight / 10)) * (tire_limit/max_tire_limit)))
			if velocity.length() < 100:
				light_controll("rear",true, 1.7)
			else:
				light_controll("rear",true, 2.0)
	else:
		steering_angle = max_steering_angle# / ((velocity.length() + 4000)/4000)
	if handbrake > 0:
		tire_limit -= tirelimit_scale * (brake_force / -50.0)
		acceleration = transform.x * ((brake_force * 0.35) * (d*0.7))
		light_controll("rear",true, 1.7)
	if backward <= 0 and handbrake <= 0:
		light_controll("rear",true, 1.25)

#for calculating how much the car is sliding
func sliding():
	var vel_dirr = velocity.angle()
	var self_dirr = self.rotation
	if d > 0:
		slide_ammount = abs(abs(vel_dirr)-abs(self_dirr))
	else: 
		slide_ammount = 0

func apply_friction():
	#If speed is too low stop the car, can cause issues for reverse if not fixed
	if velocity.length() < 25 and forward > 0 and backward <= 0 and running_dyno == 0 and forward <= 0:
		velocity = Vector2.ZERO
	var friction_force = velocity * (ground_friction / (wheels_on_road + 0.1))
	var drag_force = 0
	if engine_rpm < (max_engine_rpm * 0.98):
		drag_force = velocity * velocity.length() * drag #for smoother redline
	else:
		drag_force = velocity * velocity.length() * drag_rpm_limit
	var mechanical_friction = engine.block.tq * (engine_rpm / 10000.0) #bigger engines more brake and higher rpm more brake
	if forward <= 0 or running_dyno != 0: #Engine Brake
		if abs(velocity.x) > 1:
			acceleration.x -= velocity.x / abs(velocity.x) * mechanical_friction #opposite force of velocity
		if abs(velocity.y) > 1:
			acceleration.y -= velocity.y / abs(velocity.y) * mechanical_friction #opposite force of velocity
	acceleration += drag_force + friction_force
	max_drag_acceleration = drag_force + friction_force + Vector2(velocity.x / abs(velocity.x) * mechanical_friction, velocity.y / abs(velocity.y) * mechanical_friction)

func calculate_steering(delta):
	var rear_wheel = position - transform.x * wheelbase/2.0
	var front_wheel = position + transform.x * wheelbase/2.0
	rear_wheel += velocity * delta * drive_train[0]
	front_wheel += velocity.rotated(steer_direction) * delta * drive_train[1]
	var new_heading = (front_wheel - rear_wheel).normalized()
	var traction = traction_slow
	d = new_heading.dot(velocity.normalized()) 
	if velocity.length() > slip_speed and d > 0:
		traction = traction_fast
	elif velocity.length() > slip_speed * 2:
		traction = traction_fast / 2
	if d > 0:
		velocity = velocity.lerp(new_heading * velocity.length(), traction) #forward gear
	if d < 0 and backward > 0:
		#velocity = -new_heading * velocity.length() #reverse gear
		velocity = (velocity.lerp(-new_heading * velocity.length(), traction * 10)) #reverse gear
	elif d < 0: 
		if velocity.x > 0:
			velocity.x = velocity.x*0.99
		if velocity.y > 0:
			velocity.y = velocity.y*0.99
		if velocity.x < 0:
			velocity.x = velocity.x*0.99
		if velocity.y < 0:
			velocity.y = velocity.y*0.99
	rotation = new_heading.angle()


func traction_controller():
	#understeer when not accelerating
	var understeer_mod = abs((tire_limit/max_tire_limit)-1)/((4.5+4.5/handling_bonus)/2)+1
	var understeer_mod_fwd = abs((tire_limit/max_tire_limit)-1)/((3.5+3.5/handling_bonus)/2)+1
	var oversteer_mod_rwd = abs((tire_limit/max_tire_limit)-1)*((0.35+0.35/handling_bonus)/2)+1
	var oversteer_mod_awd = abs((tire_limit/max_tire_limit)-1)*((0.18+0.18/handling_bonus)/2)+1
	var oversteer_mod_handbreak = abs((tire_limit/max_tire_limit)-1)*0.4+1
	var gas_mod = 0.002
	var no_gas_mod = 0.001
	
	traction_fast_max = 0.12
	var drift_smoothing = 1.3 # Used to make transitions between drift and traction smoother
	traction_fast = (traction_fast_max * ((tire_limit + 10)/max_tire_limit) ** drift_smoothing)* (1-slide_ammount)
	traction_fast = clamp(traction_fast, 0.003, traction_fast_max)
	#RWD
	if drive_train_type == 0:
		#if accelerating
		if forward > 0:
			#+= for smoothly changing between throttle and no throttle
			drive_train[0] += gas_mod
			drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_rwd)
		else:
			drive_train[0] -= no_gas_mod
			drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_handbreak)
		#if not accelerating
		if handbrake > 0:
			drive_train[0] += gas_mod
			drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_handbreak)
		drive_train[1] = understeer_mod
	
	#FWD
	if drive_train_type == 1:
		if handbrake > 0:
			drive_train[0] += gas_mod
			drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_handbreak)
		if forward > 0:
			#+= for smoothly changing between throttle and no throttle
			drive_train[1] += gas_mod
			drive_train[1] = clamp(drive_train[1], 1.0 , understeer_mod_fwd)
			if handbrake <= 0:
				drive_train[0] -= no_gas_mod
				drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_handbreak)
		else:
			drive_train[1] -= no_gas_mod
			drive_train[1] = clamp(drive_train[1], understeer_mod , understeer_mod_fwd)
			if handbrake <= 0:
				drive_train[0] -= no_gas_mod
				drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_handbreak)
	#AWD
	#if accelerating
	if drive_train_type == 2:
		#if accelerating
		if forward > 0:
			#+= for smoothly changing between throttle and no throttle
			drive_train[0] += gas_mod
			drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_awd)
		else:
			drive_train[0] -= no_gas_mod
			drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_handbreak)
		#if not accelerating
		if handbrake > 0:
			drive_train[0] += gas_mod
			drive_train[0] = clamp(drive_train[0], 1.0 , oversteer_mod_handbreak)
		drive_train[1] = understeer_mod

func traction_constructor():
	#RWD constructor
	if drive_train_type == 0:
		traction_fast_max -= 0.004 #More slip with RWD
	#FWD constructor
	if drive_train_type == 1: 
		traction_fast_max += 0.012 #Less slip with FWD
	#AWD construcotr
	if drive_train_type == 2:
		traction_fast_max += 0.002 #Less slip with AWD

func get_value(): #Calculates how much car is worth by all parts combined
	var combined_parts = [engine.block, engine.internals, engine.top, engine.exhaust_manifold, engine.intake_manifold, 
		engine.air_filter, chassi, driveshaft, subframe, fenders, f_bumper, r_bumper, hood, headlights, taillights, spoiler, 
		mirrors, gearbox, radiator,exhaust,brakes,suspension,tires,wheels]
	var value : int
	for part in combined_parts:
		if part != null and part.price != null and part.durability != null: #Null check
			value += int(part.price * 0.8 * (float(part.durability) / 100.0)) #Durability damage reduces value
	
	return value

func reward_bonus():
	var value = get_value()
	if value > 0:
		var bonus = (value / 20000.0) ** 0.2 # Sets 20.000$ to be baseline (0% bonus) ** 0.2 makes it so smaller punnsihment for bellow 20.000$ and smaller reward for above
		return bonus

#region Bridge
var above_bridge = false
var bellow_bridge = true
var bridge_overwrite = false
var z_index_applied = false
func bridge_process():
	if is_loaded() == true:
		##Above bridge
		if bellow_bridge == false and above_bridge == true and z_index_applied == false: 
			self.z_index += 200
			light_controll("light_z",true, 1)
			bridge_overwrite = true
			z_index_applied = true
			set_collision_mask_value(1,false) #Don't Collide with Cars underneath
			set_collision_layer_value(1, false) #Is Collidable for other cars on bridge
			set_collision_layer_value(2, true) #Is Collidable for other cars on bridge
			set_collision_mask_value(2,true) #Collide with Guardrails
			
			set_collision_mask_value(3,false) #Don't Collide with Bridge Pillars
		##Bellow bridge
		elif bridge_overwrite == false and z_index_applied == true:
			self.z_index -= 200
			light_controll("light_z",false, 1)
			z_index_applied = false
			set_collision_mask_value(1,true) #Collide with Cars underneath
			set_collision_layer_value(1, true) #Is Collidable for other cars on bridge
			set_collision_layer_value(2, false) #Is not Collidable for other cars on bridge
			set_collision_mask_value(2,false) #Don't Collide with Guardrails
			
			set_collision_mask_value(3,true) #Don't Collide with Bridge Pillars
			


#These get imported from "Bridge Detector" in the race track
func _on_above_bridge_body_entered():
	above_bridge = true
	bridge_process()

func _on_bellow_bridge_body_entered():
	bellow_bridge = true
	bridge_process()

func _on_above_bridge_body_exited():
	above_bridge = false
	bridge_overwrite = false
	bridge_process()

func _on_bellow_bridge_body_exited():
	bellow_bridge = false
	bridge_process()
