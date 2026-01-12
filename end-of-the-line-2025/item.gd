extends Node3D
 
class_name Item

@export var max_amount : int
@export var amount : int
@export var id: int
@export var price : int
@export var in_shop : bool = false # If item is sold in shop set TRUE, this remove price from player money if player has enough and prevent despawn
@export var in_hand : bool = false # If an item is equipped
@export var item_scene : String # Set the scene of the item (used for equipping items)
@export var texture : Texture2D

#Used for ranged weapon
var ammo : int



func _init(max_amount_: int = 0, amount_: int = 0, id_: int = Id.EMPTY, texture_: Texture2D = null, in_shop_ : bool = false, in_hand_ : bool = false, 
item_scene_ : String = "", ammo_ : int = 0) -> void:
	max_amount = max_amount_
	amount = amount_
	id = id_
	texture = texture_
	in_shop = in_shop_
	in_hand = in_hand_
	item_scene = item_scene_
	ammo = ammo_


func copy() -> Item:
	var new_item = Item.new(max_amount, amount, id, texture, in_shop, in_hand, item_scene, ammo)
	return new_item

func copy_instance() -> Item:
	var instance = load(item_scene).instantiate()
	instance.max_amount = max_amount
	instance.amount = amount
	instance.id = id
	instance.texture = texture
	instance.in_shop = in_shop
	instance.in_hand = in_hand
	instance.item_scene = item_scene
	instance.ammo = ammo
	return instance
