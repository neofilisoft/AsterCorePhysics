extends Node3D

const AsterCorePhysicsScript := preload("res://addons/astercore_physics/astercore.gd")

var astercore_world: RefCounted
var vehicle_controller: RefCounted

var floor_body: RefCounted
var sphere_body: RefCounted
var vehicle_body: RefCounted

var body_to_node: Dictionary = {}
var status_label: Label3D

func _ready() -> void:
	_create_environment()
	if not AsterCorePhysicsScript.is_runtime_available():
		push_error("AsterCore runtime is not available. Check the addon binaries in addons/astercore_physics/bin.")
		return

	astercore_world = AsterCorePhysicsScript.create_world()
	astercore_world.set_gravity(Vector3(0.0, -9.81, 0.0))
	_spawn_native_world()
	_setup_vehicle()
	_update_status_text()

func _physics_process(delta: float) -> void:
	if astercore_world == null:
		return

	var throttle := Input.get_action_strength("ui_up") - Input.get_action_strength("ui_down")
	var steering := Input.get_action_strength("ui_right") - Input.get_action_strength("ui_left")
	var handbrake := Input.is_action_pressed("ui_accept")
	vehicle_controller.step(delta, throttle, steering, handbrake)

	if Input.is_action_just_pressed("ui_select"):
		sphere_body.apply_impulse(Vector3(0.0, 6.0, -12.0))

	astercore_world.step(delta, 1)
	_sync_visuals()
	_update_status_text()

func _create_environment() -> void:
	var camera := Camera3D.new()
	camera.position = Vector3(0.0, 8.0, 18.0)
	camera.look_at(Vector3(0.0, 1.5, 0.0))
	add_child(camera)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-42.0, 25.0, 0.0)
	add_child(sun)

	var floor_mesh := MeshInstance3D.new()
	floor_mesh.mesh = BoxMesh.new()
	floor_mesh.scale = Vector3(30.0, 0.4, 30.0)
	floor_mesh.position = Vector3(0.0, -1.2, 0.0)
	add_child(floor_mesh)

	status_label = Label3D.new()
	status_label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	status_label.position = Vector3(-6.0, 4.0, 0.0)
	add_child(status_label)

func _spawn_native_world() -> void:
	floor_body = astercore_world.create_box(Vector3(15.0, 1.0, 15.0), Vector3(0.0, -1.0, 0.0), false, 0.0)
	sphere_body = astercore_world.create_sphere(0.8, Vector3(0.0, 5.0, 0.0), true, 1.0)
	vehicle_body = astercore_world.create_box(Vector3(1.2, 0.4, 2.0), Vector3(0.0, 2.0, 5.0), true, 1.0)

	body_to_node[sphere_body] = _create_body_mesh("sphere", Color(0.3, 0.8, 1.0))
	body_to_node[vehicle_body] = _create_body_mesh("vehicle", Color(0.95, 0.35, 0.2))

func _setup_vehicle() -> void:
	vehicle_controller = AsterCorePhysicsScript.create_vehicle_controller(astercore_world, vehicle_body)
	vehicle_controller.aerodynamic.drag_coefficient = 0.68
	vehicle_controller.aerodynamic.reference_area = 2.8
	vehicle_controller.waterinteraction.water_level = -0.2
	vehicle_controller.waterinteraction.reference_depth = 1.4

func _create_body_mesh(kind: String, color: Color) -> Node3D:
	var mesh_instance := MeshInstance3D.new()
	mesh_instance.mesh = SphereMesh.new() if kind == "sphere" else BoxMesh.new()
	mesh_instance.material_override = StandardMaterial3D.new()
	mesh_instance.material_override.albedo_color = color
	mesh_instance.position = Vector3.ZERO
	add_child(mesh_instance)
	return mesh_instance

func _sync_visuals() -> void:
	for body in body_to_node.keys():
		var visual: Node3D = body_to_node[body]
		visual.position = body.get_position()

func _update_status_text() -> void:
	if status_label == null or astercore_world == null:
		return

	var ray: Dictionary = astercore_world.raycast(Vector3(0.0, 8.0, 0.0), Vector3.DOWN, 32.0)
	var lines := PackedStringArray()
	lines.append("AsterCore Plugin Demo")
	lines.append("Runtime ready: %s" % astercore_world.is_ready())
	lines.append("Sphere active: %s" % sphere_body.is_active())
	lines.append("Vehicle speed: %.2f" % vehicle_body.get_linear_velocity().length())
	lines.append("Ray hit body: %s" % ray["body_id"])
	lines.append("Vehicle aero: %s" % vehicle_controller.aerodynamic.to_dictionary())
	lines.append("Vehicle waterinteraction: %s" % vehicle_controller.waterinteraction.to_dictionary())
	status_label.text = "\n".join(lines)
