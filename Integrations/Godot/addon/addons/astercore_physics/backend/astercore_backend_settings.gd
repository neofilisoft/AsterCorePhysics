extends RefCounted
class_name AsterCoreBackendSettings

const SETTING_ENABLED := "astercore_physics/backend/enable_experimental_backend_bridge"
const SETTING_BACKEND_NAME := "astercore_physics/backend/backend_name"
const SETTING_NOTES := "astercore_physics/backend/notes"

static func ensure_project_settings() -> void:
	if not ProjectSettings.has_setting(SETTING_ENABLED):
		ProjectSettings.set_setting(SETTING_ENABLED, false)
	if not ProjectSettings.has_setting(SETTING_BACKEND_NAME):
		ProjectSettings.set_setting(SETTING_BACKEND_NAME, "AsterCore Physics")
	if not ProjectSettings.has_setting(SETTING_NOTES):
		ProjectSettings.set_setting(SETTING_NOTES, "Experimental PhysicsServer3DExtension bridge scaffold. The packaged addon includes a working native AsterCoreSpace3D runtime and the first backend registration layer, but a full PhysicsServer3D replacement still requires a deeper native RID/space/body implementation.")
	ProjectSettings.set_as_basic(SETTING_ENABLED, true)
	ProjectSettings.set_as_basic(SETTING_BACKEND_NAME, true)
	ProjectSettings.set_as_basic(SETTING_NOTES, false)
