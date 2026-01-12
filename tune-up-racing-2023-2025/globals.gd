extends Node

var main : Main
var selected_car : CharacterBody2D

func get_track_info(level : String) -> Dictionary:
	match level:
		"costal_circuit":
			return {
				"car_count" : 7,
				"lap_count" : 3,
				"drift_score" : 120000,
				"drift_time" : 110,
				"time_attack_time" : 52,
				"track_bouns" : 1
			}
			
		"hikari_test_track":
			return {
				"car_count" : 5,
				"lap_count" : 5,
				"drift_score" : 80000,
				"drift_time" : 90,
				"time_attack_time" : 30,
				"track_bouns" : 0.65
			}
	
	return {}

# Format numbers, 10000 -> 10.000
func format_number(num: int) -> String:
	var num_str = str(num)
	var result = ""
	var count = 0
	
	for i in range(num_str.length() - 1, -1, -1):
		result = num_str[i] + result
		count += 1
		if count % 3 == 0 and i != 0:
			result = "." + result  # Insert period as thousand separator
	
	return result
