# AsterCore Physics Godot Addon

This addon exposes a real middleware bridge for Godot 4.6+ and Redot 4.4+ through `AsterCoreCAPI` and a native `AsterCoreSpace3D` GDExtension class.

What works today:
- native world creation and stepping
- box and sphere body creation
- dynamic body activation checks
- position and linear velocity sync
- raycast queries
- higher-level GDScript wrappers for world and body handles
- demo scene and addon packaging flow

Included layers:
- `addons/astercore_physics/astercore.gd` facade for world, body, raycast, and vehicle helpers
- `addons/astercore_physics/world.gd` wrapper over the native `AsterCoreSpace3D` runtime
- `addons/astercore_physics/body.gd` lightweight body handle wrapper
- `addons/astercore_physics/vehicle/*` gameplay-side vehicle helpers that keep aerodynamic and waterinteraction settings visible in the plugin API
- `addons/astercore_physics/modules/*` capability wrappers for destruction, fluid, soft body, VBD, and GPU compute
- `demo/AsterCoreDemo3D.tscn` sample 3D scene that uses the plugin runtime directly

Quick start:
1. Build `AsterCoreCAPI` and `astercore_godot` from the AsterCore repo.
2. Copy `addons/astercore_physics` and `demo` into a Godot or Redot project.
3. Ensure `addons/astercore_physics/bin/windows/` contains `astercore_godot.dll` and `AsterCoreCAPI.dll`.
4. Enable the plugin in Project Settings > Plugins.
5. Open `demo/AsterCoreDemo3D.tscn` and run it.

Notes:
- This addon is a usable middleware bridge, not a global PhysicsServer replacement.
- `world`, `body`, and `raycast` use the native runtime directly.
- The vehicle helper keeps aerodynamic and waterinteraction settings visible in the plugin API, but it is still layered on top of the rigid body middleware bridge.
- Destruction, fluid, soft body, VBD, and GPU compute remain capability-facing wrappers until native Godot bindings for those systems are expanded.
