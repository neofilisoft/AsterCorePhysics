@tool
extends EditorPlugin

const SETTINGS_SCRIPT := preload("res://addons/astercore_physics/backend/astercore_backend_settings.gd")

func _enter_tree() -> void:
	SETTINGS_SCRIPT.ensure_project_settings()

func _exit_tree() -> void:
	pass
