extends Node2D

var engine = null
var car = null
var engines = {}
var cars = {}
var money = 0
var xp = 0
var level = 0
var time = [1,0,0] #Day, Hour, Minute
var part_inventory = {
	"car" : {
		"Front Bumper" : {},
		"Rear Bumper" : {},
		"Fenders" : {},
		"Hood" : {},
		"Mirrors" : {},
		"Headlights" : {},
		"Taillights" : {},
		"Spoiler" : {},
		"Suspension" : {},
		"Wheels" : {},
		"Tires" : {},
		"Brakes" : {},
		"Subframe" : {},
		"Driveshaft" : {},
		"Gearbox" : {},
		"Radiator" : {},
		"Exhaust" : {}
	},
	"engine" : {
		"Block" : {},
		"Internals" : {},
		"Top" : {},
		"Exhaust Manifold" : {},
		"Intake Manifold" : {},
		"Airfilter" : {}
	}
}
var file_location = "user://savegame.save"
var temp_key = null
var temp_key_car = null
var temp_key_part = null
var selected_car_key = null

var startup_save = preload("res://Save_Game/startup_save.tscn")

func set_engine(NODE):
	engine = NODE
	temp_key = engine.selected_engine_key
func set_car(NODE):
	car = NODE
	temp_key_car = car.selected_car_key


func _ready():
	
	#Set script to "not pauseable"
	process_mode = Node.PROCESS_MODE_ALWAYS
	
	pass
	if load_file("engines") != null:
		engines = load_file("engines")
	if load_file("cars") != null:
		cars = load_file("cars")
	if load_file("part_inventory") != null:
		part_inventory = load_file("part_inventory")
		print("part_inventory loaded")
	if load_file("selected_car_key") != null:
		selected_car_key = load_file("selected_car_key")
	if load_file("player_stats") != null:
		xp = load_file("player_stats").xp
		level = load_file("player_stats").level
		money = load_file("player_stats").money
	if load_file("time") != null:
		time = load_file("time")
	
	#IF SAVEFILE IS EMPTY
	if FileAccess.file_exists(file_location) == false:
		var startup_save_instanse = startup_save.instantiate()
		var file = FileAccess.open(file_location, FileAccess.WRITE)
		if file:
			# Write the JSON string to the file
			file.store_string(JSON.stringify(startup_save_instanse.save))
			file.close()
			print("NEW SAVEFILE CREATED")
			_ready()


func save_player_stats():
	if money != null:
		var part_dict = {
			"xp" : xp,
			"level" : level,
			"money" : money
		}
		return part_dict

func save_engine_stats():
	if engine != null:
		var part_dict = {
			"name" : engine.get_child(0).get_child(0).name,
			"engine_id" : engine.selected_engine,
			"key" : temp_key,
			"size" : [engine.width_left, engine.width_right, engine.lenght_front, engine.lenght_rear],
			"block" : [engine.selected_block, engine.block_durability],
			"internals" : [engine.selected_internals, engine.internals_durability],
			"top" : [engine.selected_top, engine.top_durability],
			"exhaust_manifold" : [engine.selected_exhaust_manifold, engine.exhaust_manifold_durability],
			"intake_manifold" : [engine.selected_intake_manifold, engine.intake_manifold_durability],
			"air_filter" : [engine.selected_air_filter, engine.air_filter_durability],
			"tune" : [engine.max_boost, engine.max_rpm],
			"in_car": engine.in_car,
			"rarity": engine.rarity
		}
		return part_dict
func save_car_stats():
	if car != null:
		var part_dict = {
			"key" : temp_key_car,
			"engine" : car.selected_engine,
			"name" : car.get_child(0).get_child(0).name,
			"car_id" : car.selected_car,
			"chassi" : [car.selected_chassi, car.chassi_durability, car.chassi_color],
			"driveshaft" : [car.selected_driveshaft, car.driveshaft_durability],
			"subframe" : [car.selected_subframe, car.subframe_durability],
			"fenders" : [car.selected_fenders, car.fenders_durability, car.fenders_color],
			"f_bumper" : [car.selected_f_bumper, car.f_bumper_durability, car.f_bumper_color],
			"r_bumper" : [car.selected_r_bumper, car.r_bumper_durability, car.r_bumper_color],
			"hood" : [car.selected_hood, car.hood_durability, car.hood_color],
			"headlights" : [car.selected_headlights, car.headlights_durability, car.headlights_color],
			"taillights" : [car.selected_taillights, car.taillights_durability, car.taillights_color],
			"spoiler": [car.selected_spoiler, car.spoiler_durability, car.spoiler_color],
			"mirrors" : [car.selected_mirrors, car.mirrors_durability, car.mirrors_color],
			"brakes" : [car.selected_brakes, car.brakes_durability],
			"suspension" : [car.selected_suspension, car.suspension_durability],
			"tires" : [car.selected_tires, car.tires_durability],
			"wheels" : [car.selected_wheels, car.wheels_durability],
			"gearbox" : [car.selected_gearbox, car.gearbox_durability],
			"radiator" : [car.selected_radiator, car.radiator_durability],
			"exhaust" : [car.selected_exhaust, car.exhaust_durability],
			"color" : Color(car.selected_color),
			"in_garage" : car.in_garage,
			"gear_ratio" : car.gear_ratio
		}
		return part_dict

func save_selected_key(key):
	selected_car_key = key
	return selected_car_key

func save():
	if FileAccess.file_exists(file_location):
		
		var save_dict = {"engines" : engines, "engine parts" : save_engine_stats(), 
		"cars" : cars, "car parts" : save_car_stats(),"part_inventory" : part_inventory, 
		"selected_car_key" : save_selected_key(selected_car_key), 
		"player_stats" : save_player_stats(), "time" : time}
		
		var save_game = FileAccess.open(file_location, FileAccess.WRITE)
		var json_string = JSON.stringify(save_dict)
		save_game.store_line(json_string)
	else:
		"NULL SAVE"
	
	#Also save car market stats
	CarMarket.save()

func add_engine(): #for adding engines to the players inventory
	#adds a engine to the dictionary
	var i = 0
	if engines != null:
		i = largest_key("engine", null) + 1 #+1 to set the next engine
		temp_key = i
	engines[i] = save_engine_stats()
	save() #saves the engine to a JSON file
func add_car(): #for adding cars to the players inventory
	#adds a car to the dictionary
	var i = 0
	if cars != null:
		i = largest_key("car", null) + 1 #+1 to set the next car
		temp_key_car = i
	cars[i] = save_car_stats()
	save()#saves the car to a JSON file
func inv_add(item_to_add): #for adding parts to the players inventory (works differently from car and engine)
	#adds a part to the dictionary 
	#(part format [0 = Type(0 = Car 1 = Engine), 1 = ID (what car or engine it is for), 
	#2 = Part type (what type of part it is), 3 = Part number (what part in specific it is), 
	#4 = the key it has, 5 = durability)
	#6 = Color index of part (if not paintable just leave it as 0)
	#{Type, ID, Part_Type, Part_number, Key, Durablility, Color} 
	
	var id = null #Gets defined in the if statements bellow because of different names
	if item_to_add.type == 0:
		id = item_to_add.Car_ID
	elif item_to_add.type == 1:
		id = item_to_add.Engine_ID
	
	var dict_format = {"Type" : item_to_add.type, "ID" : id, "Part_Type" : item_to_add.id[1], "Part_number" : item_to_add.Part_Number, "Key" : null, "Durability" : item_to_add.durability, "Color" : item_to_add.color}
	var i = 0
	var part = id_to_string(dict_format) #Convert int id to string names
	#Get correct key
	if part_inventory != null:
		#send correct part type directory to largest_key() function
		i = largest_key("part_inventory",part_inventory.get(part[0]).get(part[1])) + 1 #+1 to set the next item
	else:
		i = 0
	temp_key_part = i
	dict_format.Key = temp_key_part
	#Adds a part to the correct place in part inventory
	part_inventory.get(part[0]).get(part[1])[i] = dict_format #adds one item to a specified category
	save() #saves the car to a JSON fil


func remove_engine(INDEX):
	if INDEX != "0": #Can't remove empty engine form inva
		if engines == null:
			engines = load_file("engines")
		engines.erase(str(INDEX))
		save()
func remove_car(INDEX):
	cars = load_file("cars")
	cars.erase(str(INDEX))
	save()
func remove_inv(item_to_remove): #for adding parts to the players inventory (works differently from car and engine)
	#adds a part to the dictionary 
	#(part format [0 = Type(0 = Car 1 = Engine), 1 = ID (what car or engine it is for), 
	#2 = Part type (what type of part it is), 3 = Part number (what part in specific it is), 
	#4 = the key it has, 5 = durability)
	#{Type, ID, Part_Type, Part_number, Key, Durablility, Color} 
	#Check if engine or car
	var id = null
	if "Engine_ID" in item_to_remove: 
		id = item_to_remove.Engine_ID #if engine
	else:
		id = item_to_remove.Car_ID #if car
	#Get part to string form
	var dict_format = {"Type" : item_to_remove.type, "ID" : id, "Part_Type" : item_to_remove.id[1], "Part_number" : item_to_remove.Part_Number, "Key" : null, "Durability" : item_to_remove.durability, "Color" : item_to_remove.color}
	var part = id_to_string(dict_format) #Convert int id to string name
	#Find the item in part_inventory:
	var part_key : int
	for inv_part in part_inventory.get(part[0]).get(part[1]).values():
#		print("part_key ",inv_part,"   inv_part.Key ",dict_format)
		if inv_part.ID == dict_format.ID and inv_part.Part_number == dict_format.Part_number and inv_part.Durability == dict_format.Durability and inv_part.Color == dict_format.Color:
			part_key = inv_part.Key
#	print("PART IN SAVE FILE: ", part_inventory.get(part[0]).get(part[1]).get(str(part_key)),"  Key: ", part_key)#part_inventory.get(part[0]).get(part[1]).get(str(part_key)))
	part_inventory.get(part[0]).get(part[1]).erase(str(part_key)) #get the item at correct key and remove it
	part_inventory.get(part[0]).get(part[1]).erase(part_key)
	save() #saves the changes


func largest_key(category, _part_location):
	var type = null
	if category == "car":
		type = cars
	elif category == "engine":
		type = engines
	elif category == "part_inventory":
		type = _part_location
	#For loop for finding the biggest key
	var temp = 0
	for n in type.keys().size():
		var sort_var = int(type.keys()[n])
		if sort_var > temp:
			temp = sort_var
		n+=1
	return temp

func inv_check(part_to_be_checked): #checks if a part exists in the inventory
	part_inventory = load_file("part_inventory")
	var type = part_to_be_checked.type
	var part_type = part_to_be_checked.id[1]
	var part_number = part_to_be_checked.Part_Number
	var id = 402 #not assigned yet
	#Access the correct place in directory
	var formated_part_to_be_checked = {"Type" : type, "Part_number" : part_number, "Part_Type" : part_type, "ID" : null, "Durability" : 100, "Color": 0}
	var dir_location = id_to_string(formated_part_to_be_checked)
	if dir_location[0] != "empty_part":
		#print(part_to_be_checked)
		if type == 1:
			id = part_to_be_checked.Engine_ID
		elif type == 0:
			id = part_to_be_checked.Car_ID
		
		var size = part_inventory.get(dir_location[0]).get(dir_location[1]).size() #dir_location to get if car or engine and what type
		var durability = 0
		var color = 0
		var n = 0
		var i = 0
		var results = []
		while i < size:
			if part_inventory.get(dir_location[0]).get(dir_location[1]).get(str(n)) != null:
				i+=1
				var inv_part = part_inventory.get(dir_location[0]).get(dir_location[1]).get(str(n))
				if inv_part.Type == type and inv_part.Part_Type == part_type and inv_part.Part_number == part_number and inv_part.ID == id:
					durability = inv_part.Durability
					color = inv_part.Color
					results.append([durability, color])
			n += 1
		return results

 
func edit_engine(engine_node): #For editing an engine in the dictionary
	set_engine(engine_node)
	engines = load_file("engines")
	if engines != null:
		engines[str(engine.selected_engine_key)] = save_engine_stats()
	else:
		engines = save_engine_stats()
	save()
func edit_car(car_node): #For editing a car in the dictionary
	set_car(car_node)
	cars = load_file("cars")
	if cars != null:
		cars[str(car.selected_car_key)] = save_car_stats()
	else:
		cars = save_car_stats()
	save()

func load_file(filetype):
	if FileAccess.file_exists(file_location):
		var dataFile = FileAccess.open(file_location, FileAccess.READ)
		var parsedResult = JSON.parse_string(dataFile.get_as_text())
		
		if parsedResult is Dictionary:
			#for selecting what index from the dictionary in the save file we want the stats from
			if filetype == "engines":
				engines = parsedResult.get("engines")
				return parsedResult.get("engines")
			elif filetype == "engine parts":
				return parsedResult.get("engine parts")
			elif filetype == "cars":
				return parsedResult.get("cars")
			elif filetype == "car parts":
				return parsedResult.get("car parts")
			elif filetype == "part_inventory":
				if parsedResult.get("part_inventory") != {}:
					return parsedResult.get("part_inventory")
			elif filetype == "selected_car_key":
				return parsedResult.get("selected_car_key")
			elif filetype == "player_stats":
				return parsedResult.get("player_stats")
			elif filetype == "time":
				return parsedResult.get("time")
			elif filetype == "all":
				return parsedResult
		else:
			print("Error reading file")
	else:
		print("No savefile found")


func select_car(KEY):
	if KEY != null:
		var sc = load_file("cars").get(str(KEY)) #selected car
		var part_dict = {
			"key" : sc.key,
			"engine" : sc.engine,
			"name" : sc.name,
			"car_id" : sc.car_id,
			"chassi" : [sc.chassi[0],sc.chassi[1],sc.chassi[2]],
			"driveshaft" : [sc.driveshaft[0],sc.driveshaft[1]],
			"subframe" : [sc.subframe[0],sc.subframe[1]],
			"fenders" : [sc.fenders[0],sc.fenders[1],sc.fenders[2]],
			"f_bumper" : [sc.f_bumper[0],sc.f_bumper[1],sc.f_bumper[2]],
			"r_bumper" : [sc.r_bumper[0],sc.r_bumper[1],sc.r_bumper[2]],
			"hood" : [sc.hood[0],sc.hood[1],sc.hood[2]],
			"headlights" : [sc.headlights[0],sc.headlights[1],sc.headlights[2]],
			"taillights" : [sc.taillights[0],sc.taillights[1],sc.taillights[2]],
			"spoiler": [sc.spoiler[0],sc.spoiler[1],sc.spoiler[2]],
			"mirrors" : [sc.mirrors[0],sc.mirrors[1],sc.mirrors[2]],
			"brakes" : [sc.brakes[0],sc.brakes[1]],
			"suspension" : [sc.suspension[0],sc.suspension[1]],
			"tires" : [sc.tires[0],sc.tires[1]],
			"wheels" : [sc.wheels[0],sc.wheels[1]],
			"gearbox" : [sc.gearbox[0], sc.gearbox[1]],
			"radiator" : [sc.radiator[0], sc.radiator[1]],
			"exhaust" : [sc.exhaust[0], sc.exhaust[1]],
			"color" : sc.color,
			"in_garage" : sc.in_garage,
			"gear_ratio" : sc.gear_ratio
		}
		return part_dict


func select_engine(KEY):
	var sc = load_file("engines").get(str(KEY)) #selected car
	if sc != null:
		var part_dict = {
			"name" : sc.name,
			"engine_id" : sc.engine_id,
			"key" : sc.key,
			"size" : sc.size,
			"block" : [sc.block[0], sc.block[1]],
			"internals" : [sc.internals[0], sc.internals[1]],
			"top" : [sc.top[0], sc.top[1]],
			"exhaust_manifold" : [sc.exhaust_manifold[0], sc.exhaust_manifold[1]],
			"intake_manifold" : [sc.intake_manifold[0], sc.intake_manifold[1]],
			"air_filter" : [sc.air_filter[0], sc.air_filter[1]],
			"tune" : [sc.tune[0], sc.tune[1]],
			"in_car": sc.in_car,
			"rarity": sc.rarity
		}
		return part_dict
	else:
		print("Select Engine Dictionary problem")


#Takes in directory of item stats (same type as inv add) and returns an array: 
#[(car or engine) , (part category name)]
func id_to_string(item_to_add):
	var part_type_string : String
	var type_string : String
	match item_to_add.Type:
		0: #CAR
			type_string = "car"
			match item_to_add.Part_Type:
				1:
					part_type_string = "Driveshaft"
				2:
					part_type_string = "Fenders"
				3:
					part_type_string = "Front Bumper"
				4:
					part_type_string = "Headlights"
				5:
					part_type_string = "Hood"
				6:
					part_type_string = "Mirrors"
				7:
					part_type_string = "Rear Bumper"
				8:
					part_type_string = "Subframe"
				9:
					part_type_string = "Taillights"
				10:
					part_type_string = "Spoiler"
				11:
					part_type_string = "Brakes"
				12:
					part_type_string = "Suspension"
				13:
					part_type_string = "Tires"
				14:
					part_type_string = "Wheels"
				15:
					part_type_string = "Gearbox"
				16:
					part_type_string = "Radiator"
				17:
					part_type_string = "Exhaust"
		
		1: #ENGINE
			type_string = "engine"
			match item_to_add.Part_Type:
				0:
					part_type_string = "Block"
				1:
					part_type_string = "Exhaust Manifold"
				2:
					part_type_string = "Intake Manifold"
				3:
					part_type_string = "Internals"
				4:
					part_type_string = "Top"
				5:
					part_type_string = "Airfilter"
		2:
			type_string = "empty_part"
			part_type_string = "empty_part"
	
	return [type_string,part_type_string]

#DEVTOOLS
func clear_inventory():
	part_inventory = {
		"car" : {
			"Front Bumper" : {},
			"Rear Bumper" : {},
			"Fenders" : {},
			"Hood" : {},
			"Mirrors" : {},
			"Headlights" : {},
			"Taillights" : {},
			"Spoiler" : {},
			"Suspension" : {},
			"Wheels" : {},
			"Tires" : {},
			"Brakes" : {},
			"Subframe" : {},
			"Driveshaft" : {},
			"Gearbox" : {},
			"Radiator" : {},
			"Exhaust" : {}
		},
		"engine" : {
			"Block" : {},
			"Internals" : {},
			"Top" : {},
			"Exhaust Manifold" : {},
			"Intake Manifold" : {},
			"Airfilter" : {}
		}
	}
	print(part_inventory)
	save()
