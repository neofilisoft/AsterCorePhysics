# AsterCore Physics Backend Scaffold

This folder contains the first `PhysicsServer3DExtension` bridge scaffold for AsterCore.

What is included:
- `astercore_physics_server_3d.gd`: an experimental backend class that extends `PhysicsServer3DExtension`.
- `astercore_backend_settings.gd`: project-setting helper for backend bridge metadata.
- `astercore_backend_bootstrap.gd`: helper that reports backend availability and current project settings.

What is already working:
- The packaged native runtime (`AsterCoreSpace3D`) is built, bundled, and smoke-tested.
- The addon can be installed from the packaged zip.

What is still native work:
- Full RID-backed shape/body/space ownership.
- Full PhysicsServer3D method coverage.
- Early server registration at startup so the backend can fully replace the active project physics server.
- Full node/scene compatibility with `RigidBody3D`, `Area3D`, `CharacterBody3D`, joints, and direct state queries.

This scaffold is intended to keep the Godot-side bridge organized while the deeper native backend is filled in.
