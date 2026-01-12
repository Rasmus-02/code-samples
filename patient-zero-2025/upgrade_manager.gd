extends Node

const UPGRADE_SCREEN = preload("res://scenes/ui/upgrade_screen.tscn")

const ATTACKSPEEDUPGRADE = preload("res://scripts/upgrades/attack_speed_upgrade.gd")
const DETECTIONUPGRADE = preload("res://scripts/upgrades/detection_upgrade.gd")
const HEALTHREGENUPGRADE = preload("res://scripts/upgrades/health_regen_upgrade.gd")
const HEALTHUPGRADE = preload("res://scripts/upgrades/health_upgrade.gd")
const TANKUPGRADE = preload("res://scripts/upgrades/tank_upgrade.gd")
const MOVEMENTSPEEDUPGRADE = preload("res://scripts/upgrades/movement_speed_upgrade.gd")
const SPITTERUPGRADE = preload("res://scripts/upgrades/spitter_upgrade.gd")
const SUICIDEUPGRADE = preload("res://scripts/upgrades/suicide_upgrade.gd")

var avaliable_upgrades: Array[Script] = [
	ATTACKSPEEDUPGRADE,#0
	DETECTIONUPGRADE,#1
	HEALTHREGENUPGRADE,#2
	HEALTHUPGRADE,#3
	TANKUPGRADE,#4
	MOVEMENTSPEEDUPGRADE,#5
	SPITTERUPGRADE,#6
	SUICIDEUPGRADE#7
]

var upgrades: Array[Upgrade]
var upgrade_experience: int
var experience_to_next_upgrade: int = 20
var level_up_exponentiality: float = 1.1

#Fuck you figure out the number yourself :)
func create_upgrade(upgrade_number: int) -> Upgrade:
	var script: Script = avaliable_upgrades[upgrade_number]
	
	var upgrade: Node = Node.new()
	upgrade.set_script(script)
	
	var rarity: float = randf_range(0,1.0)
	upgrade.rarity = set_rarity(rarity)
	
	return upgrade

#Common 0.25
#Uncommon 0.125
#Rare 0.0725
#Epic 0.03625
#Legendary 0.018125
#Poop 0.018125
func set_rarity(rarity: float) -> String:
	if rarity > 0.25:
		return "Common"
	if rarity > 0.125:
		return "Uncommon"
	if rarity > 0.0725:
		return "Rare"
	if rarity > 0.03625:
		return "Epic"
	if rarity > 0.018125:
		return "Legendary"
	else:#haha sucks to suck idiot
		return "Poop"

func get_upgrade_value(upgrade_type, rarity : String) -> float:
	var amount : float
	if upgrade_type is AttackSpeedUpgrade:
			match rarity:
				"Common":	amount = 1.1
				"Uncommon":	amount = 1.2
				"Rare":		amount = 1.3
				"Epic":		amount = 1.4
				"Legendary":	amount = 1.5
				"Poop":		amount = 1.01
	elif upgrade_type is MovementSpeedUpgrade:
			match rarity:
				"Common":	amount = 1.1
				"Uncommon":	amount = 1.15
				"Rare":		amount = 1.2
				"Epic":		amount = 1.25
				"Legendary":	amount = 1.3
				"Poop":		amount = 1.01
	elif upgrade_type is DetectionUpgrade:
			match rarity:
				"Common":	amount = 1.1
				"Uncommon":	amount = 1.15
				"Rare":		amount = 1.2
				"Epic":		amount = 1.3
				"Legendary":	amount = 1.4
				"Poop":		amount = 1.01
	elif upgrade_type is HealthRegenUpgrade:
			match rarity: # Hp amount / second
				"Common":	amount = 0.25
				"Uncommon":	amount = 0.5
				"Rare":		amount = 1
				"Epic":		amount = 1.5
				"Legendary":	amount = 2.5
				"Poop":		amount = 0.05
	elif upgrade_type is HealthUpgrade:
			match rarity:
				"Common":	amount = 1.1
				"Uncommon":	amount = 1.2
				"Rare":		amount = 1.3
				"Epic":		amount = 1.4
				"Legendary":	amount = 1.5
				"Poop":		amount = 1.01
	elif upgrade_type is TankUpgrade:
			match rarity:
				"Common":	amount = 0.1
				"Uncommon":	amount = 0.25
				"Rare":		amount = 0.5
				"Epic":		amount = 0.75
				"Legendary":	amount = 1
				"Poop":		amount = 0.01
	elif upgrade_type is SpitterUpgrade:
			match rarity:
				"Common":	amount = 0.1
				"Uncommon":	amount = 0.25
				"Rare":		amount = 0.5
				"Epic":		amount = 0.75
				"Legendary":	amount = 1
				"Poop":		amount = 0.01
	elif upgrade_type is SuicideUpgrade:
			match rarity:
				"Common":	amount = 0.1
				"Uncommon":	amount = 0.25
				"Rare":		amount = 0.5
				"Epic":		amount = 0.75
				"Legendary":	amount = 1
				"Poop":		amount = 0.01
	return amount

func gain_experience(amount: int) -> void:
	upgrade_experience += amount
	if upgrade_experience >= experience_to_next_upgrade:
		#Save extra exp and add towards next levelup
		var exp_remainder = upgrade_experience - experience_to_next_upgrade
		level_up()
		upgrade_experience += exp_remainder

func level_up() -> void:
	#increase exp needed to level up each time
	experience_to_next_upgrade *= level_up_exponentiality
	upgrade_experience = 0
	show_level_up_screen()

func show_level_up_screen() -> void:
	# Never 2 same cards
	
	#Generate 3 different numbers used for upgrades
	var indices = []
	var max_index = avaliable_upgrades.size() - 1
	while indices.size() < 3:
		var r = randi_range(0, max_index)
		if r not in indices:
			indices.append(r)
	
	
	var ugs: Array[Upgrade] = [
		create_upgrade(indices[0]),
		create_upgrade(indices[1]),
		create_upgrade(indices[2])
	]
	
	var ins = UPGRADE_SCREEN.instantiate()
	Globals.ui.add_child(ins)
	ins.set_upgrades(ugs)
	get_tree().paused = true

func aquire_upgrade(upgrade: Upgrade) -> void:
	upgrades.append(upgrade)
	if upgrade.is_unit_upgrade:
		for agent: Agent in Globals.agents.get_children():
			if agent.team == agent.INFECTED:
				agent.add_upgrade(upgrade)
	else:
		upgrade.upgrade(Globals.player_agent)
