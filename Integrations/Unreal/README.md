# AsterCore Physics for Unreal Engine

This plugin is a middleware integration for Unreal Engine 4 and 5. It does not replace Chaos globally. Instead, it gives you runtime components that create and step an AsterCore world and sync actor transforms through the native `AsterCoreCAPI.dll` bridge.

## Install

1. Copy `Integrations/Unreal` into your Unreal project's `Plugins/AsterCorePhysics` directory.
2. Build `AsterCoreCAPI.dll` from the AsterCore repo.
3. Copy `AsterCoreCAPI.dll` into `Plugins/AsterCorePhysics/Binaries/ThirdParty/Win64/`.
4. Regenerate project files.
5. Enable the plugin and recompile the project.

## Runtime Components

- `UAsterCoreWorldComponent` creates and steps an AsterCore world.
- `UAsterCoreBodyComponent` creates a box or sphere body and syncs the owning actor transform.

## Scope

This plugin is intended as a practical middleware bridge for gameplay integration. It is not a full Chaos replacement plugin.
