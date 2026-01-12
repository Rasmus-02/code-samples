extends Node3D

var peer = ENetMultiplayerPeer.new()
@export var player_scene : PackedScene
@export var ip = "localhost"
@export var port = 22222


func _on_host_pressed() -> void:
	print("HOST")
	hide_buttons()
	#setup_upnp()
	
	# Start the server
	var error = peer.create_server(int(port))
	if error != OK:
		print("Failed to create server:", error)
		return
	
	multiplayer.multiplayer_peer = peer
	multiplayer.peer_connected.connect(_add_player)
	_add_player(1) #Add host as player


func _on_join_pressed() -> void:
	print("JOIN")
	
	# Create client connection and handle errors 
	var error = peer.create_client(ip, int(port))
	if error != OK:
		print("Failed to connect to server: ", error)
		return
	
	await get_tree().process_frame  # Wait one frame
	
	hide_buttons()
	multiplayer.multiplayer_peer = peer
	print("Client connected! My Peer ID: ", multiplayer.get_unique_id())


func _add_player(id):
	print("Adding player with ID: ", id)

	var player = player_scene.instantiate()
	player.name = "player_" + str(id)
	player.set_multiplayer_authority(player.name.to_int())
	$Network/Players.add_child(player)
	
	player.global_position = global_position

## Setup upnp
func setup_upnp() -> void:
	var upnp = UPNP.new()
	var discover_result = upnp.discover()
	
	if discover_result == UPNP.UPNP_RESULT_SUCCESS:
		if upnp.get_gateway() and upnp.get_gateway().is_valid_gateway():
			var map_result_udp = upnp.add_port_mapping(port,port,"godot_udp","UDP",0)
			var map_result_tcp = upnp.add_port_mapping(port,port,"godot_udp","TCP",0)
			upnp.add_port_mapping(port,port, "godot_udp")
			if not map_result_udp == UPNP.UPNP_RESULT_SUCCESS:
				upnp.add_port_mapping(port,port,"","UDP")
			if not map_result_tcp == UPNP.UPNP_RESULT_SUCCESS:
				upnp.add_port_mapping(port,port,"","TCP")
	
	upnp.delete_port_mapping(port, "UDP")
	upnp.delete_port_mapping(port, "TCP")


func hide_buttons():
	$Camera3D/HBoxContainer.hide()
