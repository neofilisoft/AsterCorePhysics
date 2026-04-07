@tool
extends Resource
class_name AsterCoreAerodynamicSettings

@export var enabled: bool = true
@export var drag_coefficient: float = 0.55
@export var lift_coefficient: float = 0.05
@export var reference_area: float = 2.2
@export var air_density: float = 1.225
@export var forward_axis: Vector3 = Vector3.FORWARD

func to_dictionary() -> Dictionary:
	return {
		"enabled": enabled,
		"drag_coefficient": drag_coefficient,
		"lift_coefficient": lift_coefficient,
		"reference_area": reference_area,
		"air_density": air_density,
		"forward_axis": forward_axis,
	}
