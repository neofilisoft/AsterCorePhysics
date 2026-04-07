extends RefCounted
class_name AsterCoreBackendBootstrap

const SETTINGS := preload("res://addons/astercore_physics/backend/astercore_backend_settings.gd")
const BACKEND_SCRIPT_PATH := "res://addons/astercore_physics/backend/astercore_physics_server_3d.gd"

static func describe_backend_status() -> Dictionary:
	SETTINGS.ensure_project_settings()
	return {
		"bridge_enabled": ProjectSettings.get_setting(SETTINGS.SETTING_ENABLED, false),
		"backend_name": ProjectSettings.get_setting(SETTINGS.SETTING_BACKEND_NAME, "AsterCore Physics"),
		"physics_engine_setting": ProjectSettings.get_setting("physics/3d/physics_engine", "DEFAULT"),
		"runtime_class_available": ClassDB.class_exists("AsterCoreSpace3D"),
		"backend_script": BACKEND_SCRIPT_PATH,
	}
