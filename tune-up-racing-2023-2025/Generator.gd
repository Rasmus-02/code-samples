class_name generator
extends Node
var difficulty = Settings.get_difficulty_bonus("AI Difficulty")
var max_rarity : int
var race_status : bool

#Main Functions
func generate_car(rarity, weight, torque_estimate, grip, downforce, brake_force, precision, race : bool):
	#If generating AI and not purchaseable cars
	race_status = race
	print("DIFF ",difficulty)
	if race == true:
		rarity = rarity_to_int(rarity)
		max_rarity = rarity
		torque_estimate = torque_estimate * difficulty
		grip *= difficulty
		downforce *= difficulty
		brake_force *= difficulty
		# If medium or harder difficulty, encounter more challanging cars
		if Settings.difficulty >= 2:
			max_rarity += 1
	else:
		max_rarity = rarity
	
	#Find Suitable Chassi
	var car_array = []
	var universal = AssetList.car_list.get_child(0)
	for i in AssetList.car_list.get_child_count()-1: #-1 and +1 is to offset universal part list
		#Make sure chassi in range for rarity and weight (75% is the estimated chassi weight of the total car)
		var car = AssetList.car_list.get_child(i+1)
		var instance = car.chassi[0].instantiate()
		# If in race +-1 rarity, if in shop, max unlocked rarity
		var instance_rarity = rarity_to_int(instance.rarity)
		if (race_status == true and instance_rarity <= max_rarity and instance_rarity >= max_rarity - 2) or (instance_rarity <= max_rarity and race_status == false):
			if instance.weight > (weight * precision) * 0.70 and instance.weight < (weight / precision) * 0.75:
				car_array.append(car)
		instance.queue_free()
	if car_array.size() != 0:
		randomize()
		var rng = randi_range(0, car_array.size() - 1) #Pick A Random Chassi from the array
		var car_model = car_array[rng]
		
		#Find proper parts for the chassi
		var chassi = car_model.chassi[0].instantiate()
		var driveshaft = _find_suitable_part("driveshaft", car_model.driveshaft, null, null, null)
		var gearbox = _find_suitable_part("gearbox", universal.gearbox, [driveshaft.drivetrain,driveshaft.drivetrain], null, null)
		var fenders = _find_suitable_part("fenders", car_model.fenders, null, [grip, (grip / precision)*1.2], null)
		var wheels = _find_suitable_part("wheels", universal.wheels, null, [0,fenders.max_tire_width], null)
		var tires = _find_suitable_part("tires", universal.tires,  [grip * precision, (grip / precision)*1.2], [0,wheels.max_tire_width], null)
		var f_bumper = _find_suitable_part("f_bumper", car_model.f_bumper, [0, (downforce)], null, null)
		var spoiler = _find_suitable_part("spoiler", car_model.spoiler, [(downforce * 0.8 - f_bumper.downforce) * precision, (downforce *1.2 - f_bumper.downforce) / precision], null, null)
		var headlight = _find_suitable_part("headlights", car_model.headlights, null, null, null)
		var taillights = _find_suitable_part("taillights", car_model.taillights, null, null, null)
		var mirrors = _find_suitable_part("mirrors", car_model.mirrors, null, null, null)
		var subframe = _find_suitable_part("subframe", car_model.subframe, null, null, null)
		var exhaust = _find_suitable_part("exhaust", car_model.exhaust, null, null, null)
		var brakes = _find_suitable_part("brakes", universal.brakes, [brake_force * precision , brake_force / precision], null, null)
		var suspension = _find_suitable_part("suspension", universal.suspension, null, null, null)
		var radiator = _find_suitable_part("radiator", universal.radiator, null, null, null)
		
		
		var car_weight = chassi.weight + driveshaft.weight + gearbox.weight + fenders.weight + wheels.weight + tires.weight + spoiler.weight + f_bumper.weight + headlight.weight + taillights.weight + mirrors.weight + subframe.weight + exhaust.weight + brakes.weight + suspension.weight + radiator.weight
		#print(car_weight)
		var r_bumper = _find_suitable_part("r_bumper", car_model.r_bumper, [0, weight - car_weight], null, null)
		#print(r_bumper.weight)
		var hood = _find_suitable_part("hood", car_model.hood, [0, weight - car_weight], null, null)
		#print(hood.weight)
		car_weight += hood.weight + r_bumper.weight
		
		var car_price = chassi.price + driveshaft.price + gearbox.price + fenders.price + wheels.price + tires.price + spoiler.price + f_bumper.price + headlight.price + taillights.price + mirrors.price + subframe.price + exhaust.price + brakes.price + suspension.price + radiator.price + hood.price + r_bumper.price
		
		var drivetrain_loss = (1 - driveshaft.drivetrain_loss) * exhaust.tq_mod_exhaust
		var engine_position_offset = Vector2.ZERO
		var engine_bay_size = [chassi.engine_bay_lenght - radiator.width, chassi.engine_bay_width]
		if driveshaft.drivetrain == 1: #FWD 
			engine_position_offset = Vector2(gearbox.get_node("Engine").position.y + driveshaft.get_node("Gearbox").position.x - chassi.engine_bay_start_lenght, driveshaft.get_node("Gearbox").position.y - gearbox.get_node("Engine").position.x)
		else: #RWD AWD
			engine_position_offset = Vector2(gearbox.get_node("Engine").position.x + driveshaft.get_node("Gearbox").position.x - chassi.engine_bay_start_lenght, driveshaft.get_node("Gearbox").position.y - gearbox.get_node("Engine").position.y)
		
		var parts = [chassi, driveshaft, subframe, fenders, f_bumper, r_bumper, hood, headlight, taillights, spoiler, mirrors, gearbox, radiator,exhaust,brakes,suspension,tires,wheels]
		
		var part_dict = {"Car_ID" : driveshaft.Car_ID,
		"price" : car_price,
		"weight" : car_weight,
		"reliability" : chassi.reliability,
		"gearbox_type" : gearbox.gearbox_type,
		"position_offset" : engine_position_offset,
		"engine_bay_size" : engine_bay_size,
		"stock_engine" : car_model.stock_engine,
		"drivetrain_loss" : drivetrain_loss,
		"drivetrain" : driveshaft.drivetrain,
		"driveshaft" : driveshaft.Part_Number,
		"gearbox" : gearbox.Part_Number,
		"fenders" : fenders.Part_Number,
		"wheels" : wheels.Part_Number,
		"tires" : tires.Part_Number,
		"spoiler" : spoiler.Part_Number,
		"f_bumper" : f_bumper.Part_Number,
		"r_bumper" : r_bumper.Part_Number,
		"headlight" : headlight.Part_Number,
		"taillights" : taillights.Part_Number,
		"mirrors" : mirrors.Part_Number,
		"hood" : hood.Part_Number,
		"subframe" : subframe.Part_Number,
		"exhaust" : exhaust.Part_Number,
		"brakes" : brakes.Part_Number,
		"suspension" : suspension.Part_Number,
		"radiator" : radiator.Part_Number,}
		
		for part in parts:
			part.queue_free()
		parts.clear()
		
		return part_dict

func generate_engine(rarity, weight, car_weight, torque, engine_bay_size, engine_offset, drivetrain, stock_engine, precision, race : bool):
	#If generating AI and not purchaseable cars
	race_status = race
	if race == true:
		torque *= difficulty
	else:
		max_rarity = rarity
	
	randomize()
	var max_lenght
	var max_width_l
	var max_width_r
	if drivetrain == 1: #if FWD
		max_width_l = engine_offset.x
		max_width_r = (engine_bay_size[0]) - engine_offset.x
		max_lenght = (engine_bay_size[1]*0.5) + engine_offset.y
	else:
		max_width_l = (engine_bay_size[1]*0.5) + engine_offset.y
		max_width_r = (engine_bay_size[1]*0.5) - engine_offset.y
		max_lenght = (engine_bay_size[0]) - engine_offset.x
	
	#Find Suitable Engine
	var engine_array = []
	var universal = AssetList.engine_list.get_child(0)
	var max_weight = weight - car_weight #The max weight for the engine
	var stock_or_swapped = randi_range(0, 10)
	#Use Stock Engine
	if stock_or_swapped <= 8:
		var engine = AssetList.engine_list.get_child(stock_engine) #get engine at index specified in car part list
		engine_array.append(engine)
	#Find a swappable engine
	else:
		for i in AssetList.engine_list.get_child_count()-1: #-1 and +1 is to offset universal part list
			#Make sure engine in range weight (15% is the estimated chassi weight of the total car), and that there is a block that can handle the torque
			var engine = AssetList.engine_list.get_child(i+1)
			var instance = engine.block[1].instantiate()
			# If in race +-1 rarity, if in shop, max unlocked rarity
			var instance_rarity = rarity_to_int(instance.rarity)
			if (race_status == true and instance_rarity <= max_rarity and instance_rarity >= max_rarity - 2) or (instance_rarity <= max_rarity and race_status == false):
				if instance.lenght < max_lenght and instance.width * 0.5 < max_width_l and instance.width * 0.5 < max_width_r:
					engine_array.append(engine)
			instance.queue_free()
	if engine_array.size() != 0:
		var rng = randi_range(0, engine_array.size() - 1) #Pick A Random Chassi from the array
		var engine_model = engine_array[rng]
		
		var est_tq = 0 #Estimated Torque
		var est_tq_nb = 0 #Estimated Torque Without boost
		var boost_estimate_total = 0.0
		var boost_estimate_turbo = 0.0
		var block = _find_suitable_part("block", engine_model.block, [torque, 9999], null, null)
		est_tq += block.tq
		est_tq_nb += block.tq
		var intake_manifold = _find_suitable_part("intake_manifold", engine_model.intake_manifold, [0,torque], [0,max_width_l - block.width * 0.5], est_tq)
		if intake_manifold.supercharger == true:
			boost_estimate_total += (intake_manifold.supercharer_displacement_capacity * 0.0004)
			est_tq += (intake_manifold.supercharer_displacement_capacity * 0.0004)
		est_tq *= intake_manifold.tq_mod
		est_tq_nb *= intake_manifold.tq_mod
		var exhaust_manifold = _find_suitable_part("exhaust_manifold", engine_model.exhaust_manifold, [torque * precision, torque / precision], [0,max_width_r - block.width * 0.5], [est_tq, torque, est_tq_nb, intake_manifold.layout])
		if exhaust_manifold.turbo == true:
			boost_estimate_turbo = _estimate_boost(est_tq, est_tq_nb, torque, exhaust_manifold.get_turbo_max_size(), exhaust_manifold.turbo_efficiency)
			boost_estimate_total += boost_estimate_turbo
			est_tq += est_tq * (boost_estimate_turbo * (exhaust_manifold.get_turbo_max_size()/70)*exhaust_manifold.turbo_efficiency) #Estimate turbo power
		est_tq *= exhaust_manifold.tq_mod
		est_tq_nb *= exhaust_manifold.tq_mod
		var internals = _find_suitable_part("internals", engine_model.internals, [torque / precision, 9999], [boost_estimate_total, 9999], est_tq)
		est_tq = est_tq * (internals.compression / 10)
		est_tq_nb = est_tq_nb * (internals.compression / 10)
		boost_estimate_total += internals.compression
		var top = _find_suitable_part("top", engine_model.top, [est_tq * precision, est_tq / precision], [boost_estimate_total, 9999], est_tq)
		est_tq *= top.tq_mod
		est_tq_nb *= top.tq_mod
		#Set air filter position to wanted position
		var air_filter_layout = intake_manifold.layout
		if exhaust_manifold.turbo == true:
			air_filter_layout = "front"
		var air_filter = _find_suitable_part("air_filter", universal.air_filter, null, [air_filter_layout, air_filter_layout], null)
		est_tq *= air_filter.tq_mod
		est_tq_nb *= air_filter.tq_mod
		
		#Increasing the accuracy of turbocharged engines
		if exhaust_manifold.turbo == true:
			#For adding back supercharger and compression
			var tq_sc_nb = est_tq_nb
			tq_sc_nb = est_tq_nb * (internals.compression / 10)
			if intake_manifold.supercharger == true:
				tq_sc_nb += (intake_manifold.supercharer_displacement_capacity * 0.0004)
			boost_estimate_turbo = _estimate_boost(tq_sc_nb ,est_tq_nb, torque, exhaust_manifold.get_turbo_max_size(), exhaust_manifold.turbo_efficiency)
			tq_sc_nb += est_tq_nb * (boost_estimate_turbo * (exhaust_manifold.get_turbo_max_size()/70)*exhaust_manifold.turbo_efficiency) #Estimate turbo power
			est_tq = tq_sc_nb
		
		var est_power = estimate_torque(block, air_filter, exhaust_manifold, intake_manifold, internals, top, boost_estimate_turbo)
		
		var engine_price = block.price + exhaust_manifold.price + intake_manifold.price + internals.price + air_filter.price + top.price
		var engine_weight = block.weight + exhaust_manifold.weight + intake_manifold.weight + internals.weight + air_filter.weight + top.weight
		var parts = [block, internals, top, intake_manifold, exhaust_manifold, air_filter]
		var part_dict = {"Engine_ID" : block.Engine_ID,
		"price" : engine_price,
		"reliability" : block.reliability,
		"weight" : engine_weight,
		"fuel_type" : top.fuel_type,
		"max_boost" : boost_estimate_turbo,
		"max_rpm" : top.max_hp_rpm,
		"Tq" : est_power.tq,
		"Hp" : est_power.hp,
		"block" : block.Part_Number,
		"intake_manifold" : intake_manifold.Part_Number,
		"exhaust_manifold" : exhaust_manifold.Part_Number,
		"internals" : internals.Part_Number,
		"top" : top.Part_Number,
		"air_filter" : air_filter.Part_Number,}
		
		
		#If any engine part is not within size limits, return null to generate new engine
		#block
		if block.lenght > max_lenght: #check so part doesn't fits in engine bay
			for part in parts:
				part.queue_free()
			parts.clear()
			return(null)
		#V ENGINES
		if block.layout == "V": #different calc for V engines 
			#Exhaust Manifold
			if block.width * 0.5 + exhaust_manifold.width > max_width_l or block.width * 0.5 + exhaust_manifold.width > max_width_r or block.lenght + exhaust_manifold.lenght > max_lenght: #check so part doesn't fits in engine bay
				for part in parts:
					part.queue_free()
				parts.clear()
				return(null)
			#Intake Manifold
			if block.lenght + intake_manifold.lenght + air_filter.lenght > max_lenght: #check so part doesn't fits in engine bay
				for part in parts:
					part.queue_free()
				parts.clear()
				return(null)
			#Air Filter
			if block.lenght + air_filter.lenght + intake_manifold.lenght > max_lenght: #check so part doesn't fits in engine bay
				for part in parts:
					part.queue_free()
				parts.clear()
				return(null)
		#Other Engines
		else:
			#Exhaust Manifold
			if block.width * 0.5 + exhaust_manifold.width > max_width_r or block.lenght + exhaust_manifold.lenght > max_lenght: #check so part doesn't fits in engine bay
				for part in parts:
					part.queue_free()
				parts.clear()
				return(null)
			#Intake Manifold
			if block.width * 0.5 + intake_manifold.width > max_width_l or block.lenght + intake_manifold.lenght > max_lenght: #check so part doesn't fits in engine bay
				for part in parts:
					part.queue_free()
				parts.clear()
				return(null)
		
		#If engine is not within torque limits return NULL to generate new one
		if est_tq < torque * precision or est_tq > torque / precision:
			for part in parts:
				part.queue_free()
			parts.clear()
			return null
		
		#If engine is not withing weight limits return NULL to generate new one
		if engine_weight < max_weight * (precision * 0.8) or engine_weight > max_weight / (precision * 0.8):
			for part in parts:
				part.queue_free()
			parts.clear()
			return null
		
		
		#If engine is knocking return NULL to generate new one
		if boost_estimate_total > top.max_compression:
			for part in parts:
				part.queue_free()
			parts.clear()
			return null
		
		#If engine fufills requirements
		for part in parts:
			part.queue_free()
		parts.clear()
		return part_dict



#Side functions used by main
func _find_suitable_part(category ,category_array, stat1_range, stat2_range, stat_import):  #Model is the Car or Engine model, Category is the type of part, Stat to check is the stat that we want
	randomize()
	var temp_array = []
	var temp_backup_array = []
	var stat_1
	var stat_2
	var instance
	#For loop for finding parts that fit the criterias
	for i in category_array.size() - 1:
		#Get correct stat
		match category:
			"driveshaft", "gearbox":
				instance = category_array[i+1].instantiate()
				stat_1 = instance.drivetrain
			"fenders":
				#Chanse if not AI CAR to be empty
				if race_status == false and randi_range(0,10) > 8:
					instance = category_array[0].instantiate()
				elif race_status == false and randi_range(0,10) < 8:
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				stat_2 = instance.max_tire_width * 5
			"wheels":
				instance = category_array[i+1].instantiate()
				stat_2 = instance.max_tire_width #Get aproximate grip capacity of rims
			"tires":
				instance = category_array[i+1].instantiate()
				stat_1 = instance.grip
				stat_2 = instance.width
			"spoiler", "f_bumper":
				#Chanse if not AI CAR to be empty
				if race_status == false and randi_range(0,10) > 6:
					instance = category_array[0].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				stat_1 = instance.downforce
			"headlights", "taillights", "mirrors", "subframe", "exhaust", "suspension":
				#Chanse if not AI CAR to be empty
				if race_status == false and randi_range(0,10) > 8:
					instance = category_array[0].instantiate()
				elif race_status == false and randi_range(0,10) < 8:
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
			"hood", "r_bumper":
				#Chanse if not AI CAR to be empty
				if race_status == false and randi_range(0,10) > 8:
					instance = category_array[0].instantiate()
				elif race_status == false and randi_range(0,10) < 8:
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				stat_1 = instance.weight
			"brakes":
				#Chanse if not AI CAR to be empty
				if race_status == false and randi_range(0,10) > 8:
					instance = category_array[0].instantiate()
				elif race_status == false and randi_range(0,10) < 8:
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				stat_1 = instance.brake_force
			"radiator":
				#Chanse if not AI CAR to be empty
				if race_status == false and randi_range(0,10) > 8:
					instance = category_array[0].instantiate()
				elif race_status == false and randi_range(0,10) < 8:
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
			"block":
				if race_status == false and randi_range(0,10) < 8: # Reduce chance to be modified
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				stat_1 = instance.max_tq
			"intake_manifold":
				if race_status == false and randi_range(0,10) < 8: # Reduce chance to be modified
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				if instance.supercharger == true:
					stat_1 = stat_import * (instance.supercharer_displacement_capacity * 0.0004) #Estimate supercharger power
					stat_1 *= instance.tq_mod
				else:
					stat_1 = stat_import * instance.tq_mod
				stat_2 = instance.width
			"exhaust_manifold":
				if race_status == false and randi_range(0,10) < 8: # Reduce chance to be modified
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				if instance.turbo == true:
					#Estimate Boost Level
					var max_boost_estimate = _estimate_boost(stat_import[0], stat_import[2], stat_import[1], instance.get_turbo_max_size(), instance.turbo_efficiency)
					stat_1 = stat_import[0]
					stat_1 += stat_import[2] * (max_boost_estimate * (instance.get_turbo_max_size()/70)*instance.turbo_efficiency) #Estimate turbo power
					stat_1 *= instance.tq_mod
				else:
					stat_1 = stat_import[0] * instance.tq_mod
				if stat_import[3] == "top" and instance.turbo == true: #If carb and turbo make it illegal (by pretending to be huge so it won't fit)
					stat_2 = 9999
				else:
					stat_2 = instance.width
			"internals":
				if race_status == false and randi_range(0,10) < 8: # Reduce chance to be modified
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				stat_1 = instance.max_tq
				var stock_internals = category_array[1].instantiate()
				stat_2 = instance.compression + 2.5 - stock_internals.compression
			"top":
				if race_status == false and randi_range(0,10) < 8: # Reduce chance to be modified
					instance = category_array[1].instantiate()
				else:
					instance = category_array[i+1].instantiate()
				var stock_top = category_array[1].instantiate()
				var rev_hp_boost = instance.max_hp_rpm / stock_top.max_hp_rpm
				stat_1 = stat_import + instance.tq_mod * rev_hp_boost
				stat_2 = instance.max_compression
			"air_filter":
				instance = category_array[i+1].instantiate()
				stat_2 = instance.layout
		if (stat1_range == null or stat_1 >= stat1_range[0] and stat_1 <= stat1_range[1]) and (stat2_range == null or stat_2 >= stat2_range[0] and stat_2 <= stat2_range[1]):
			if race_status == true or rarity_to_int(instance.rarity) <= max_rarity: #IF generating car for shop, account for rarity
				temp_array.append(instance)
		elif (stat2_range == null or stat_2 >= stat2_range[0] and stat_2 <= stat2_range[1]):
			if (stat1_range == null or stat_1 * 1.5 >= stat1_range[0] and stat_1 * 0.5 <= stat1_range[1]):
				if race_status == true or rarity_to_int(instance.rarity) <= max_rarity: #IF generating car for shop, account for rarity
					temp_backup_array.append(instance)
	var rng = randi_range(0, temp_array.size()-1)
	
	#Backup array to prevent crash
	if temp_array.size() == 0 and temp_backup_array.size() != 0:
		rng = randi_range(0, temp_backup_array.size()-1)
		clear_instances(category_array ,temp_backup_array[rng])
		return(temp_backup_array[rng])
	
	#Backup 2 just equips stock part
	if temp_array.size() == 0:
		if category_array.size() > 1:
			clear_instances(category_array ,category_array[1])
			return(category_array[1].instantiate())
		else:
			clear_instances(category_array ,category_array[0])
			return(category_array[0].instantiate())
	
	clear_instances(category_array ,temp_array[rng])
	return(temp_array[rng])

func clear_instances(list, ignore):
	for i in list:
		if i != ignore and not (i is PackedScene):
			i.queue_free()

func _estimate_boost(tq_boost, tq, goal_tq, turbo_size, turbo_efficiency):
	var max_boost_estimate = 0
	var temp_max_boost = 0.0
	var temp_power = tq_boost
	#print(turbo_size/70)
	while max_boost_estimate == 0:
		temp_max_boost += 0.1
		temp_power = tq_boost
		temp_power += tq * (temp_max_boost * (turbo_size/70)*turbo_efficiency)
		if temp_power >= goal_tq or temp_max_boost >= 3:
			max_boost_estimate = temp_max_boost
			return max_boost_estimate

func estimate_torque(block, airfilter, exhaust_manifold, intake_manifold, internals, top, boost):
	var max_horsepower_rpm = durability_perfromance(top.max_hp_rpm / 1.2, top.durability)
	var air_filter = airfilter
	var turbo = exhaust_manifold.turbo
	var supercharger = intake_manifold.supercharger
	var compression = durability_perfromance(internals.compression, internals.durability)
	var max_compression = durability_perfromance(top.max_compression, top.durability) * durability_perfromance(intake_manifold.max_compression_modifier, intake_manifold.durability)
	var max_boost = boost
	var turbo_size = exhaust_manifold.get_turbo_max_size()
	var supercharger_pulley_size = 70.0
	var supercharer_displacement_capacity = intake_manifold.supercharer_displacement_capacity
	var turbo_efficiency = exhaust_manifold.turbo_efficiency
	var max_torque = (durability_perfromance(block.tq, block.durability) * durability_perfromance(intake_manifold.tq_mod, intake_manifold.durability) 
	* durability_perfromance(exhaust_manifold.tq_mod, exhaust_manifold.durability) * durability_perfromance(top.tq_mod, top.durability) * durability_perfromance(air_filter.tq_mod, air_filter.durability))
	
	var temp_tq
	var temp_hp
	#Run twice, first time for hp, second time for tq
	var run1_hp
	var run1_tq
	
	for i in 1:
		var temp_rpm = max_horsepower_rpm
		if i == 1:
			temp_rpm = max_horsepower_rpm * 0.77
		var temp_boost = max_boost
		var temp_airflow_post_turbo = 0
		var temp_airflow_post_supercharger = 0
		
		#Turbo Calc
		if turbo == true:
			temp_airflow_post_turbo = (((turbo_size/60.0)**(0.8)) * temp_boost) * 0.8 * air_filter.tq_mod
		
		if supercharger == true:
			var pulley_size = supercharger_pulley_size / 35.0
			var loss_rate = 1.2
			var supercharger_loss = ((((max_horsepower_rpm / pulley_size) - temp_rpm) * loss_rate) / (max_horsepower_rpm / pulley_size)) * (supercharer_displacement_capacity * 0.0004)
			var temp_boost_supercharger = (temp_rpm / (max_horsepower_rpm / pulley_size)) * (supercharer_displacement_capacity * 0.0004) + supercharger_loss
			temp_boost_supercharger = clamp(temp_boost_supercharger, 0, (max_horsepower_rpm / pulley_size))
			temp_boost += temp_boost_supercharger
			temp_airflow_post_supercharger = temp_boost_supercharger * 2 * air_filter.tq_mod
		
		var temp_airflow_post = temp_airflow_post_supercharger + (temp_airflow_post_turbo * turbo_efficiency)
		
		
		temp_tq = max_torque * (float(temp_rpm) / float(max_horsepower_rpm)) + (max_torque / 8.0)
		temp_tq = (temp_tq + (temp_tq * temp_airflow_post))/2
		temp_tq = ((temp_tq * 2 - temp_tq * (temp_rpm / max_horsepower_rpm))/2.0) * 3.5 #powerloss due to friction
		temp_tq = temp_tq * (float(compression) / 10.0) #apply compression boost
		
		if compression + temp_boost > max_compression:
			temp_tq += (max_compression - (compression + temp_boost)) * (50 * (1+temp_boost)) #if over max compression start loosing power due to knock, lose more with more boost
		if turbo == true: #losses due to turbo restriction
			temp_tq -= (((turbo_size/50.0)**(2.0)) * 6) / turbo_efficiency #*10 is constant, efficiensy makes less power loss
		if supercharger == true:
			temp_tq -= supercharer_displacement_capacity / 22.0
		temp_tq = clamp(temp_tq, 0, 9999)
		temp_hp = (temp_tq * temp_rpm) / 7127.0
		
		if i == 0:
			run1_tq = temp_tq
			run1_hp = temp_hp
	
	## If test with max rpm went better, use that
	temp_hp = run1_hp
	
	return {"tq" : int(temp_tq), "hp" : int(temp_hp)}

func rarity_to_int(rarity):
	match rarity:
		"common":
			return 0
		"uncommon":
			return 1
		"rare":
			return 2
		"epic":
			return 3
		"legendary":
			return 4

func durability_perfromance(stat, durability):
	return (stat + stat * (durability * 0.0025)) * 0.8

func _generate_gear_ratio(gears, top_speed_estimate):
	pass
