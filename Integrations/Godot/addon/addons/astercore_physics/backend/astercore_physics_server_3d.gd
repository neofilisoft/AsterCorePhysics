extends PhysicsServer3DExtension
class_name AsterCorePhysicsServer3D

var _runtime: Object = null

func _init() -> void:
	if ClassDB.class_exists("AsterCoreSpace3D"):
		_runtime = ClassDB.instantiate("AsterCoreSpace3D")

func _init_ext() -> void:
	if _runtime != null and _runtime.has_method("reset_world"):
		_runtime.reset_world()

func _finish() -> void:
	_runtime = null

func _sync() -> void:
	pass

func _flush_queries() -> void:
	pass

func _end_sync() -> void:
	pass

func _step(delta: float) -> void:
	if _runtime != null:
		_runtime.step(delta, 1)

func _get_process_info(process_info: int) -> int:
	return 0

func _free_rid(rid: RID) -> void:
	pass

func _space_create() -> RID:
	return RID()

func _area_create() -> RID:
	return RID()

func _body_create() -> RID:
	return RID()

func _soft_body_create() -> RID:
	return RID()

func _shape_create(shape_type: int) -> RID:
	return RID()

func _joint_create() -> RID:
	return RID()

func _space_set_active(space: RID, active: bool) -> void:
	pass

func _space_is_active(space: RID) -> bool:
	return true

func _space_get_param(space: RID, param: int) -> float:
	return 0.0

func _space_set_param(space: RID, param: int, value: float) -> void:
	pass

func _body_set_space(body: RID, space: RID) -> void:
	pass

func _body_get_space(body: RID) -> RID:
	return RID()

func _body_set_mode(body: RID, mode: int) -> void:
	pass

func _body_get_mode(body: RID) -> int:
	return PhysicsServer3D.BODY_MODE_RIGID

func _body_add_shape(body: RID, shape: RID, transform: Transform3D, disabled: bool) -> void:
	pass

func _body_set_state(body: RID, state: int, value: Variant) -> void:
	pass

func _body_get_state(body: RID, state: int) -> Variant:
	return null

func _body_apply_central_impulse(body: RID, impulse: Vector3) -> void:
	pass

func _body_apply_impulse(body: RID, impulse: Vector3, position: Vector3) -> void:
	pass

func _shape_set_data(shape: RID, data: Variant) -> void:
	pass

func _shape_get_type(shape: RID) -> int:
	return PhysicsServer3D.SHAPE_BOX

func _shape_get_data(shape: RID) -> Variant:
	return null
