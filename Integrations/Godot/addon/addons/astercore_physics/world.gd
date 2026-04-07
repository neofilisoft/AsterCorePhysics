@tool
extends RefCounted
class_name AsterCoreWorld

const BodyScript := preload("res://addons/astercore_physics/body.gd")

var _space: Object

func _init() -> void:
	if not ClassDB.class_exists("AsterCoreSpace3D"):
		push_error("AsterCoreSpace3D is not available. Verify the native GDExtension binaries are present.")
		return
	_space = ClassDB.instantiate("AsterCoreSpace3D")

func is_ready() -> bool:
	return _space != null and _space.is_runtime_ready()

func get_native_space() -> Object:
	return _space

func reset_world() -> void:
	if is_ready():
		_space.reset_world()

func step(delta: float, collision_steps: int = 1) -> void:
	if is_ready():
		_space.step(delta, collision_steps)

func set_gravity(gravity: Vector3) -> void:
	if is_ready():
		_space.set_gravity(gravity)

func get_gravity() -> Vector3:
	return _space.get_gravity() if is_ready() else Vector3.ZERO

func create_box(half_extents: Vector3, position: Vector3, dynamic: bool = true, mass: float = 1.0) -> RefCounted:
	if not is_ready():
		return null
	var body_id: int = _space.create_box(half_extents, position, dynamic, mass)
	return BodyScript.new(self, body_id, "box")

func create_sphere(radius: float, position: Vector3, dynamic: bool = true, mass: float = 1.0) -> RefCounted:
	if not is_ready():
		return null
	var body_id: int = _space.create_sphere(radius, position, dynamic, mass)
	return BodyScript.new(self, body_id, "sphere")

func destroy_body(body_or_id: Variant) -> void:
	if not is_ready():
		return
	var body_id := _resolve_body_id(body_or_id)
	if body_id >= 0:
		_space.destroy_body(body_id)

func has_body(body_or_id: Variant) -> bool:
	if not is_ready():
		return false
	var body_id := _resolve_body_id(body_or_id)
	return body_id >= 0 and _space.has_body(body_id)

func is_body_active(body_or_id: Variant) -> bool:
	if not is_ready():
		return false
	var body_id := _resolve_body_id(body_or_id)
	return body_id >= 0 and _space.is_body_active(body_id)

func get_body_position(body_or_id: Variant) -> Vector3:
	if not is_ready():
		return Vector3.ZERO
	var body_id := _resolve_body_id(body_or_id)
	return _space.get_body_position(body_id) if body_id >= 0 else Vector3.ZERO

func set_body_position(body_or_id: Variant, position: Vector3) -> void:
	if not is_ready():
		return
	var body_id := _resolve_body_id(body_or_id)
	if body_id >= 0:
		_space.set_body_position(body_id, position)

func get_body_linear_velocity(body_or_id: Variant) -> Vector3:
	if not is_ready():
		return Vector3.ZERO
	var body_id := _resolve_body_id(body_or_id)
	return _space.get_body_linear_velocity(body_id) if body_id >= 0 else Vector3.ZERO

func set_body_linear_velocity(body_or_id: Variant, velocity: Vector3) -> void:
	if not is_ready():
		return
	var body_id := _resolve_body_id(body_or_id)
	if body_id >= 0:
		_space.set_body_linear_velocity(body_id, velocity)

func apply_body_impulse(body_or_id: Variant, impulse: Vector3) -> void:
	if not is_ready():
		return
	var body_id := _resolve_body_id(body_or_id)
	if body_id >= 0:
		_space.apply_body_impulse(body_id, impulse)

func raycast(origin: Vector3, direction: Vector3, max_distance: float = 1000.0) -> Dictionary:
	if not is_ready():
		return {"hit": false, "body_id": -1, "position": Vector3.ZERO}
	var body_id: int = _space.raycast(origin, direction, max_distance)
	var hit_position: Vector3 = _space.get_last_raycast_position()
	return {
		"hit": body_id >= 0,
		"body_id": body_id,
		"position": hit_position,
	}

func _resolve_body_id(body_or_id: Variant) -> int:
	if typeof(body_or_id) == TYPE_INT:
		return int(body_or_id)
	if body_or_id != null and body_or_id.has_method("get_id"):
		return int(body_or_id.get_id())
	return -1
