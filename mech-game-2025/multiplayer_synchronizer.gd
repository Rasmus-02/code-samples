extends MultiplayerSynchronizer
var root : Node

func _ready() -> void:
	root = get_tree().root.get_node("Main").get_node("Network").get_node("Players")

func _process(delta: float) -> void:
	
	## Sync up other players visuals to client
	if root != null:
		var players = root.get_children()
		for player in players:
			if player.name != get_parent().name:
				# Update animations
				player._handle_animation(delta)
				# Update particle effects
