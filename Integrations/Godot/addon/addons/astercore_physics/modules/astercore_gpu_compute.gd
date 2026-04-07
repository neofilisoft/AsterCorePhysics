@tool
extends RefCounted
class_name AsterCoreGpuComputeAPI

func get_capability_name() -> String:
	return "gpu_compute"

func supported() -> bool:
	return false

func summary() -> String:
	return "GPU compute scaffolding exists in the engine, but the current Godot plugin layer does not expose a native GPU compute backend yet."

func create_job_stub(job_name: String) -> Dictionary:
	return {
		"supported": supported(),
		"job_name": job_name,
		"message": summary(),
	}
