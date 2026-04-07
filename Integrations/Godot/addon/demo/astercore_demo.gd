extends Node3D

var astercore := AsterCoreSpace3D.new()
var floor_id := 0
var sphere_id := 0

func _ready() -> void:
    floor_id = astercore.create_box(Vector3(10.0, 1.0, 10.0), Vector3(0.0, -1.0, 0.0), false, 0.0)
    sphere_id = astercore.create_sphere(0.5, Vector3(0.0, 2.0, 0.0), true, 1.0)
    astercore.set_body_linear_velocity(sphere_id, Vector3(0.0, -5.0, 0.0))

func _physics_process(delta: float) -> void:
    astercore.step(delta, 1)
    if sphere_id != 0:
        global_position = astercore.get_body_position(sphere_id)
