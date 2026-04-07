@tool
extends RefCounted
class_name AsterCoreVehicleController

const AerodynamicSettingsScript := preload("res://addons/astercore_physics/vehicle/astercore_aerodynamic_settings.gd")
const WaterInteractionSettingsScript := preload("res://addons/astercore_physics/vehicle/astercore_waterinteraction_settings.gd")

var _world: RefCounted
var _body: RefCounted

var engine_force: float = 28.0
var steering_force: float = 9.0
var side_slip_damping: float = 2.5

var aerodynamic: Resource = AerodynamicSettingsScript.new()
var waterinteraction: Resource = WaterInteractionSettingsScript.new()

func configure(world: RefCounted, body: RefCounted) -> void:
	_world = world
	_body = body

func set_body(body: RefCounted) -> void:
	_body = body

func get_body() -> RefCounted:
	return _body

func step(delta: float, throttle: float, steering: float, handbrake: bool = false) -> void:
	if _world == null or _body == null or not _body.is_valid():
		return

	var velocity: Vector3 = _body.get_linear_velocity()
	var planar_velocity := Vector3(velocity.x, 0.0, velocity.z)
	var drag_impulse := Vector3.ZERO

	if aerodynamic.enabled:
		var speed := planar_velocity.length()
		if speed > 0.001:
			var drag_strength: float = 0.5 * aerodynamic.air_density * aerodynamic.drag_coefficient * aerodynamic.reference_area * speed * speed
			drag_impulse = -planar_velocity.normalized() * drag_strength * delta

	if waterinteraction.enabled:
		var depth := max(waterinteraction.water_level - _body.get_position().y, 0.0)
		if depth > 0.0 and waterinteraction.reference_depth > 0.0:
			var submergence: float = clamp(depth / waterinteraction.reference_depth, 0.0, 1.0)
			var buoyancy: Vector3 = Vector3.UP * waterinteraction.buoyancy_scale * submergence * delta
			var water_drag: Vector3 = -velocity * waterinteraction.drag_scale * submergence * delta
			_body.apply_impulse(buoyancy + water_drag)

	var drive_impulse := Vector3(throttle * engine_force * delta, 0.0, 0.0)
	var steer_impulse := Vector3(0.0, 0.0, steering * steering_force * delta)
	var damping_impulse := Vector3(-velocity.x * side_slip_damping * delta, 0.0, 0.0)
	if handbrake:
		damping_impulse += -planar_velocity * 5.0 * delta

	_body.apply_impulse(drive_impulse + steer_impulse + damping_impulse + drag_impulse)
