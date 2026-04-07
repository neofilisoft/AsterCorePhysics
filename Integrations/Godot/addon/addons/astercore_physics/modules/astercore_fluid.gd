@tool
extends RefCounted
class_name AsterCoreFluidAPI

func get_capability_name() -> String:
	return "fluid"

func supported() -> bool:
	return false

func summary() -> String:
	return "Particle fluid exists in the engine, but the current Godot plugin layer does not expose a native fluid world yet."

func create_emitter_stub(particle_count: int) -> Dictionary:
	return {
		"supported": supported(),
		"particle_count": particle_count,
		"message": summary(),
	}
