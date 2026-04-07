@tool
extends RefCounted
class_name AsterCoreSoftBodyAPI

func get_capability_name() -> String:
	return "soft_body"

func supported() -> bool:
	return false

func summary() -> String:
	return "Soft body is available in the engine, but the current Godot plugin layer has not exposed native soft body authoring yet."

func create_cloth_stub(width: int, height: int) -> Dictionary:
	return {
		"supported": supported(),
		"resolution": Vector2i(width, height),
		"message": summary(),
	}
