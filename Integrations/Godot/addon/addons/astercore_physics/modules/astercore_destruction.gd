@tool
extends RefCounted
class_name AsterCoreDestructionAPI

func get_capability_name() -> String:
	return "destruction"

func supported() -> bool:
	return false

func summary() -> String:
	return "The core engine contains destruction systems, but the current Godot plugin layer does not expose native destruction controls yet."

func create_prefractured_asset_stub(asset_name: String) -> Dictionary:
	return {
		"supported": supported(),
		"asset_name": asset_name,
		"message": summary(),
	}
