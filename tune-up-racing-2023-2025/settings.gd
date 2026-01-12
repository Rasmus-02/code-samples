extends Node

var file_location = "user://settings.save"
# var resolution_settings = {0 : Vector2(850, 480) ,1 : Vector2(1280, 720), 2 : Vector2(1920, 1080), 3 : Vector2(2560, 1440)}

#Difficulty
var difficulty : int = 0 # 0 - 4

#General
var camera_mode : int = 0 # 0 = static || 1 = dynamic
var display_mode : int # 0 = fullscreen || 1 = windowed
#var resolution = 0 # the index in resolution settings
var vsync : int = 1 # 0 = off || 1 = on

#Audio
var general_volume : float = 1.0 # 0 - 1
var car_volume : float = 1.0 # 0 - 1
var music_volume : float = 1.0 # 0 - 1
var ambient_volume : float = 1.0 # 0 - 1

func _ready():
	#Set script to "not pauseable"
	process_mode = Node.PROCESS_MODE_ALWAYS
	
	#If file doesn't exist create new file
	if FileAccess.file_exists(file_location) == false:
		print("NEW SETTINGS FILE CREATED")
		var _file = FileAccess.open(file_location, FileAccess.WRITE)
		save_settings()
	#Load the file
	load_file()
	await get_tree().create_timer(1).timeout
	apply_settings()

func save_settings():
	if FileAccess.file_exists(file_location):
		var save_dict = {"difficulty" : difficulty,"camera_mode" : camera_mode, "display_mode" : display_mode,
		"vsync" : vsync, "general_volume" : general_volume, "car_volume" : car_volume, 
		"music_volume" : music_volume, "ambient_volume" : ambient_volume}
		var save_game = FileAccess.open(file_location, FileAccess.WRITE)
		var json_string = JSON.stringify(save_dict)
		save_game.store_line(json_string)
	else:
		"ERROR: NULL SETTINGS FILE LOCATION"
	
	apply_settings()

func load_file():
	if FileAccess.file_exists(file_location):
		var dataFile = FileAccess.open(file_location, FileAccess.READ)
		var parsedResult = JSON.parse_string(dataFile.get_as_text())
		
		#Difficulty
		difficulty = parsedResult.difficulty
		#General
		camera_mode = parsedResult.camera_mode
		display_mode = parsedResult.display_mode
		#resolution = parsedResult.resolution
		vsync = parsedResult.vsync
		#Audio
		general_volume = parsedResult.general_volume
		car_volume = parsedResult.car_volume
		music_volume = parsedResult.music_volume
		ambient_volume = parsedResult.ambient_volume

func apply_settings():
	#Display Mode --------------------------------------------------------------
	#First stup correct monitor
	var primary_screen = DisplayServer.get_primary_screen()
	var current_screen = DisplayServer.get_keyboard_focus_screen()
	var screen_position = DisplayServer.screen_get_position(current_screen)
	var screen_size = DisplayServer.screen_get_size(primary_screen)
	DisplayServer.window_set_position(screen_position)
	#Set the display mode
	match display_mode:
		0: #Fullscreen
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)
			DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, false)
			DisplayServer.window_set_position(screen_position)
		#1: 
		#	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
		#	DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, true)
		#	DisplayServer.window_set_position(screen_position)
		#	DisplayServer.window_set_size(screen_size)
		1: #Windowed
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
			DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, false)
			var window_size = DisplayServer.window_get_size()
			var center_position = Vector2i(
				screen_position.x + (screen_size.x - window_size.x) / 2, 
				screen_position.y + (screen_size.y - window_size.y) / 2)
			DisplayServer.window_set_position(center_position)
			DisplayServer.window_set_size(screen_size / 2)
	
	#V-Sync --------------------------------------------------------------------
	match vsync:
		0:
			DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
		1:
			DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_ENABLED)
		2:
			DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_ADAPTIVE)
	
	#Volume --------------------------------------------------------------------
	AudioServer.set_bus_volume_db(0, linear_to_db(general_volume))
	AudioServer.set_bus_volume_db(1, linear_to_db(car_volume) - 8)
	AudioServer.set_bus_volume_db(2, linear_to_db(ambient_volume))
	AudioServer.set_bus_volume_db(3, linear_to_db(music_volume))
	
	AudioServer.set_bus_volume_db(1, -80)
	AudioServer.set_bus_volume_db(2, -80)

func get_difficulty_bonus(type : String) -> float:
	match type:
		"Money Bonus":
			match difficulty:
				0: # Beginner
					return 0.5 
				1: # Easy
					return 0.75 
				2: # Medium
					return 1.0
				3: # Hard
					return 1.25 
				4: # Insane
					return 2.0 
		"Xp Bonus":
			match difficulty:
				0: # Beginner
					return 0.5 
				1: # Easy
					return 0.75 
				2: # Medium
					return 1.0
				3: # Hard
					return 1.25 
				4: # Insane
					return 2.0
		"AI Difficulty":
			match difficulty:
				0: # Beginner
					return 0.65 
				1: # Easy
					return 0.8 
				2: # Medium
					return 1.0
				3: # Hard
					return 1.25 
				4: # Insane
					return 1.5
		"Time Bonus":
			match difficulty:
				0: # Beginner
					return 1.5
				1: # Easy
					return 1.2
				2: # Medium
					return 1.0
				3: # Hard
					return 0.8
				4: # Insane
					return 0.5
	printerr("WRONG DIFFICULTY SELECTED")
	return 0.0
