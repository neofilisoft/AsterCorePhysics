@tool
extends RefCounted
class_name AsterCoreBody

var _world: RefCounted
var _body_id: int = -1
var _shape_hint: String = ""

func _init(world: RefCounted = null, body_id: int = -1, shape_hint: String = "") -> void:
	_world = world
	_body_id = body_id
	_shape_hint = shape_hint

func get_id() -> int:
	return _body_id

func get_shape_hint() -> String:
	return _shape_hint

func is_valid() -> bool:
	return _world != null and _body_id >= 0 and _world.has_body(_body_id)

func is_active() -> bool:
	return is_valid() and _world.is_body_active(_body_id)

func get_position() -> Vector3:
	return _world.get_body_position(_body_id) if is_valid() else Vector3.ZERO

func set_position(position: Vector3) -> void:
	if is_valid():
		_world.set_body_position(_body_id, position)

func get_linear_velocity() -> Vector3:
	return _world.get_body_linear_velocity(_body_id) if is_valid() else Vector3.ZERO

func set_linear_velocity(velocity: Vector3) -> void:
	if is_valid():
		_world.set_body_linear_velocity(_body_id, velocity)

func apply_impulse(impulse: Vector3) -> void:
	if is_valid():
		_world.apply_body_impulse(_body_id, impulse)

func destroy() -> void:
	if is_valid():
		_world.destroy_body(_body_id)
	_body_id = -1
