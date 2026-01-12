extends Node

# 1 Day = 24 minutes || 1 Hour = 1 minute || 1 Minute = 60 Seconds
var second : float # Used for counting up the minutes
# Minutes and hours used for in-game time calculation for the lighting system
var minute : int 
var hour : int = 20
var day : int = 1

func _ready():
	minute = Save_Load.time[2]
	hour = Save_Load.time[1]
	day = Save_Load.time[0]

func _physics_process(delta):
	second += delta * 60
	
	if second >= 60:
		CarMarket.countdown -= 1
		second = 0
		minute += 1
	if minute >= 60:
		minute = 0
		hour += 1
	if hour >= 24:
		hour = 0
		day += 1
	if day > 7:
		day = 1
	
	Save_Load.time = [day, hour, minute]
