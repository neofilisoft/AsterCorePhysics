@tool
extends RefCounted
class_name AsterCorePhysics

const WorldScript := preload("res://addons/astercore_physics/world.gd")
const VehicleControllerScript := preload("res://addons/astercore_physics/vehicle/astercore_vehicle_controller.gd")
const AerodynamicSettingsScript := preload("res://addons/astercore_physics/vehicle/astercore_aerodynamic_settings.gd")
const WaterInteractionSettingsScript := preload("res://addons/astercore_physics/vehicle/astercore_waterinteraction_settings.gd")
const DestructionScript := preload("res://addons/astercore_physics/modules/astercore_destruction.gd")
const FluidScript := preload("res://addons/astercore_physics/modules/astercore_fluid.gd")
const SoftBodyScript := preload("res://addons/astercore_physics/modules/astercore_soft_body.gd")
const VbdScript := preload("res://addons/astercore_physics/modules/astercore_vbd.gd")
const GpuComputeScript := preload("res://addons/astercore_physics/modules/astercore_gpu_compute.gd")

static func is_runtime_available() -> bool:
	return ClassDB.class_exists("AsterCoreSpace3D")

static func create_world() -> RefCounted:
	return WorldScript.new()

static func create_vehicle_controller(world: RefCounted, body: RefCounted = null) -> RefCounted:
	var controller := VehicleControllerScript.new()
	controller.configure(world, body)
	return controller

static func create_aerodynamic_settings() -> Resource:
	return AerodynamicSettingsScript.new()

static func create_waterinteraction_settings() -> Resource:
	return WaterInteractionSettingsScript.new()

static func destruction_api() -> RefCounted:
	return DestructionScript.new()

static func fluid_api() -> RefCounted:
	return FluidScript.new()

static func soft_body_api() -> RefCounted:
	return SoftBodyScript.new()

static func vbd_api() -> RefCounted:
	return VbdScript.new()

static func gpu_compute_api() -> RefCounted:
	return GpuComputeScript.new()
