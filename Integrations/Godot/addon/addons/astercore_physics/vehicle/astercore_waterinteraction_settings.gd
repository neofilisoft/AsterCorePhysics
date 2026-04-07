@tool
extends Resource
class_name AsterCoreWaterInteractionSettings

@export var enabled: bool = true
@export var water_level: float = 0.0
@export var reference_depth: float = 1.5
@export var buoyancy_scale: float = 18.0
@export var drag_scale: float = 2.0
@export var lateral_drag_scale: float = 1.25

func to_dictionary() -> Dictionary:
	return {
		"enabled": enabled,
		"water_level": water_level,
		"reference_depth": reference_depth,
		"buoyancy_scale": buoyancy_scale,
		"drag_scale": drag_scale,
		"lateral_drag_scale": lateral_drag_scale,
	}
