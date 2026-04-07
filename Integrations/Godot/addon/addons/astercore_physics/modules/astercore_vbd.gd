@tool
extends RefCounted
class_name AsterCoreVBDAPI

func get_capability_name() -> String:
	return "vbd"

func supported() -> bool:
	return false

func summary() -> String:
	return "VBD exists in the engine tree, but the current Godot plugin layer does not expose a native VBD solver yet."

func create_solver_stub() -> Dictionary:
	return {
		"supported": supported(),
		"message": summary(),
	}
