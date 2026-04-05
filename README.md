# AsterCore Physics
Physics SDK (Core fork from Jolt)

A multi core friendly rigid body physics and collision detection library. Suitable for games and VR applications.

|[![Ragdoll Pile](https://img.youtube.com/vi/pwyCW0yNKMA/hqdefault.jpg)](https://www.youtube.com/watch?v=pwyCW0yNKMA)|
|:-|
|*A YouTube video showing a ragdoll pile simulated with AsterCore Physics.*|

For more demos and [videos](https://www.youtube.com/watch?v=pwyCW0yNKMA&list=PLYXVwtOr1CBxbA50jVg2dKUQvHW_5OOom) go to the [Samples](Docs/Samples.md) section.

## Design considerations

Why create yet another physics engine? Firstly, it has been a personal learning project. Secondly, I wanted to address some issues that I had with existing physics engines:

* Games do more than simulating physics. These things happen across multiple threads. We emphasize on concurrently accessing physics data outside of the main simulation update:
	* Sections of the simulation can be loaded / unloaded in the background. We prepare a batch of physics bodies on a background thread without locking or affecting the simulation. We insert the batch into the simulation with a minimal impact on performance.
	* Collision queries can run parallel to adding / removing or updating a body. If a change to a body happened on the same thread, the change will be immediately visible. If the change happened on another thread, the query will see a consistent before or after state. An alternative would be to have a read and write version of the world. This prevents changes from being visible immediately, so we avoid this.
	* Collision queries can run parallel to the main physics simulation. We do a coarse check (broad phase query) before the simulation step and do fine checks (narrow phase query) in the background. This way, long running processes (like navigation mesh generation) can be spread out across multiple frames.
* Accidental wake up of bodies cause performance problems when loading / unloading content. Therefore, bodies will not automatically wake up when created. Neighboring bodies will not be woken up when bodies are removed. This can be triggered manually if desired.
* The simulation runs deterministically. You can replicate a simulation to a remote client by merely replicating the inputs to the simulation. Read the [Deterministic Simulation](https://jrouwe.github.io/JoltPhysics/#deterministic-simulation) section to understand the limits.
* We try to simulate behavior of rigid bodies in the real world but make approximations. Therefore, this library should mainly be used for games or VR simulations.

## Features

* Simulation of rigid bodies of various shapes using continuous collision detection:
	* Sphere
	* Box
	* Capsule
	* Tapered-capsule
	* Cylinder
	* Tapered-cylinder
	* Convex hull
	* Plane
	* Compound
	* Mesh (triangle)
	* Terrain (height field)
* Simulation of constraints between bodies:
	* Fixed
	* Point
	* Distance (including springs)
	* Hinge
	* Slider (also called prismatic)
	* Cone
	* Rack and pinion
	* Gear
	* Pulley
	* Smooth spline paths
	* Swing-twist (for humanoid shoulders)
	* 6 DOF
* Motors to drive the constraints.
* Collision detection:
	* Casting rays.
	* Testing shapes vs shapes.
	* Casting a shape vs another shape.
	* Broadphase only tests to quickly determine which objects may intersect.
* Sensors (trigger volumes).
* Animated ragdolls:
	* Hard keying (kinematic only rigid bodies).
	* Soft keying (setting velocities on dynamic rigid bodies).
	* Driving constraint motors to an animated pose.
	* Mapping a high detail (animation) skeleton onto a low detail (ragdoll) skeleton and vice versa.
* Game character simulation (capsule)
	* Rigid body character. Moves during the physics simulation. Cheapest option and most accurate collision response between character and dynamic bodies.
	* Virtual character. Does not have a rigid body in the simulation but simulates one using collision checks. Updated outside of the physics update for more control. Less accurate interaction with dynamic bodies.
* Vehicles
	* Wheeled vehicles.
	* Tracked vehicles.
	* Motorcycles.
* Soft body simulation (e.g. a soft ball or piece of cloth).
	* Edge constraints.
	* Dihedral bend constraints.
	* Cosserat rod constraints (an edge with an orientation that can be used to orient geometry, e.g. a plant leaf).
	* Tetrahedron volume constraints.
	* Long range attachment constraints (also called tethers).
	* Limiting the simulation to stay within a certain range of a skinned vertex.
	* Internal pressure.
	* Collision with simulated rigid bodies.
	* Collision tests against soft bodies.
* Water buoyancy calculations.
* An optional double precision mode that allows large worlds.

# AsterCore Physics SDK

AsterCore Physics is a multi-core friendly rigid body physics and collision detection library for games, simulation, and middleware integration work. The runtime is built around deterministic simulation, scalable job scheduling, and an SDK-oriented toolchain that can be embedded into custom engines and external tooling.

## Design Goals

AsterCore is designed for teams that need more than a closed simulation loop. Modern games stream worlds, run queries in parallel, author destruction offline, replay state during rollback, and integrate physics into larger engine workflows. The library emphasizes:

- multi-threaded simulation and queries
- deterministic behavior within documented limits
- background-friendly content preparation
- runtime and tooling paths suitable for SDK distribution
- engine integration through CMake packages, bindings, and native bridges

## Features

- Rigid body simulation with continuous collision detection
  - sphere
  - box
  - capsule and tapered capsule
  - cylinder and tapered cylinder
  - convex hull
  - plane
  - compound
  - mesh
  - terrain / height field
- Constraint system
  - fixed
  - point
  - distance and spring constraints
  - hinge
  - slider
  - cone
  - rack and pinion
  - gear
  - pulley
  - path / spline constraints
  - swing-twist
  - 6 DOF
- Collision and queries
  - ray casts
  - shape casts
  - shape-vs-shape tests
  - broadphase-only overlap queries
  - sensors / trigger volumes
- Character simulation
  - rigid body character
  - virtual character
- Vehicles
  - wheeled vehicles
  - tracked vehicles
  - motorcycles
  - aerodynamic force helpers
  - water interaction helpers
- Soft body simulation
  - cloth and soft body primitives
  - bend, volume, tether, and skinned constraints
  - soft body contact callbacks
- Additional runtime systems
  - destruction foundations for pre-fractured assets and collapse workflows
  - snapshot / rollback foundations
  - particle fluid simulation
  - VBD tuning utilities
  - GPU compute scheduling foundation
  - C API and integration scaffolding

## AsterCore 3.0 Highlights

1. AsterCore 3.0 expands the project from a physics runtime into a more complete SDK. The core simulation stack is still the backbone, but the repository now includes SDK-facing tooling, exportable CMake packages, and installable artifacts that are easier to hand off to engine, tools, and gameplay teams.

2. The new ACD layer acts as an AsterCore debugger foundation. It captures frame timing, memory usage, determinism checksums, counters, markers, and event streams into structured output that can be archived, inspected offline, or wired into future visualization tools. This is not just for debugging isolated tests; it is intended to support engine integration, profiling, and regression tracking workflows.

3. Fracture authoring has been pushed forward through the new Fractal Editor foundation. SDK users can now author Voronoi, clustered Voronoi, and impact-driven fracture profiles, choose material-driven breakup behavior such as wood-like long shards or glass-like tiny fragments, define wake-up thresholds for static-to-dynamic transitions, and control simplified chunk hierarchy settings for destruction LOD and far-field cleanup.

4. The packaging story has improved significantly. AsterCore 3.0 is structured to be consumed as middleware, with installable headers, libraries, documentation, samples, exported CMake targets, CLI tooling, and integration bridges. This makes it much easier to embed the runtime into custom engines or downstream tools without treating the repository itself as the only supported distribution format.

## Tooling And SDK Modules

- `Tools/ACD`
  - debugger capture foundation
  - counters, markers, memory, timing, determinism capture
  - CLI export path for offline analysis
- `Tools/FractalEditor`
  - fracture authoring profiles
  - Voronoi and clustered fracture presets
  - impact-point driven authoring
  - material-based fracture shaping
  - chunk hierarchy and wake threshold metadata
- `Tools/Editor`
  - asset export helpers
  - destructible asset metadata
  - fracture authoring data export
- `Bindings`
  - C API and integration-oriented native bridges
- `Integrations`
  - engine-facing scaffolding for downstream embedding work

## Documentation

Start here:

- `HelloWorld/HelloWorld.cpp`
- `Docs/Architecture.md`
- `Docs/Samples.md`
- `Docs/ReleaseNotes.md`
- `Docs/APIChanges.md`
- `Docs/ACD.md`
- `Docs/FractalEditor.md`

If you install the SDK, the staged documentation and samples are placed under:

- `share/AsterCore/docs`
- `share/AsterCore/samples`

## Building

- C++17
- CMake-based build
- Works as a static or shared library
- Exported CMake package available through installed SDK artifacts

Typical middleware integration flow:

1. Build and install the SDK.
2. Point `CMAKE_PREFIX_PATH` at the installed SDK root.
3. Use `find_package(AsterCore CONFIG REQUIRED)`.
4. Link against `AsterCore::AsterCore`.
5. Ship runtime binaries from `bin/` when using shared builds.

## Folder Structure

- `AsterCore`
  - core runtime source
- `Assets`
  - assets used by samples, viewer, and test framework
- `Build`
  - build helpers and packaging support
- `Docs`
  - documentation
- `HelloWorld`
  - minimal example
- `Integrations`
  - integration scaffolding
- `Samples`
  - sample application and scenario coverage
- `Tools`
  - editor, debugger, and fracture authoring utilities
- `UnitTests`
  - automated test coverage

## Middleware Positioning

AsterCore is structured to be embedded into custom engines and tools rather than only used as a monolithic sample application. The repository contains the pieces needed to distribute:

- developer SDKs with headers, libraries, CMake package config, docs, and samples
- runtime payloads for shared-library deployment
- editor and authoring support for destruction-heavy workflows
- native bridges for language and engine integration

## License

The project is distributed under the MIT license. See `LICENSE`.


#### Notes

- Keep `BUILD_SHARED_LIBS=ON` to produce a dynamic library.
- Disable samples, viewer, tests, and extra tooling when you only want the SDK runtime for engine integration.
- On Windows, the repository is configured to emit `AsterCore.dll` without the MinGW `lib` prefix so the output is easier to consume from Feliss.

## Performance

If you're interested in how AsterCore scales with multiple CPUs and compares to other physics engines, take a look at [this document](https://jrouwe.nl/jolt/JoltPhysicsMulticoreScaling.pdf).

## Folder structure

* Assets - This folder contains assets used by the TestFramework, Samples and AsterCoreViewer.
* Build - Contains everything needed to build the library, see the [Build](Build/README.md) section.
* Docs - Contains documentation for the library.
* HelloWorld - A simple application demonstrating how to use the AsterCore Physics library.
* AsterCore - All source code for the library is in this folder.
* AsterCoreViewer - It is possible to record the output of the physics engine using the DebugRendererRecorder class (a .jor file), this folder contains the source code to an application that can visualize a recording. This is useful for e.g. visualizing the output of the PerformanceTest from different platforms. Currently available on Windows, macOS and Linux.
* PerformanceTest - Contains a simple application that runs a [performance test](Docs/PerformanceTest.md) and collects timing information.
* Samples - This contains the sample application, see the [Samples](Docs/Samples.md) section. Currently available on Windows, macOS and Linux.
* TestFramework - A rendering framework to visualize the results of the physics engine. Used by Samples and AsterCoreViewer. Currently available on Windows, macOS and Linux.
* UnitTests - A set of unit tests to validate the behavior of the physics engine.

## Bindings for other languages

* C [here](https://github.com/amerkoleci/joltc), [here](https://github.com/zig-gamedev/zphysics/tree/main/libs/JoltC) and [here](https://github.com/SecondHalfGames/JoltC/)
* [C#](https://github.com/amerkoleci/JoltPhysicsSharp)
* [Java or Kotlin](https://stephengold.github.io/jolt-jni-docs)
* [JavaScript](https://github.com/jrouwe/JoltPhysics.js)
* [Rust](https://github.com/SecondHalfGames/jolt-rust)
* [Zig](https://github.com/zig-gamedev/zphysics)

## Integrations in other engines

* [Godot](https://github.com/godotengine/godot)
* [Source Engine](https://github.com/Joshua-Ashton/VPhysics-AsterCore)
* Unreal Plugin [here](https://github.com/OversizedSunCoreDev/ArtilleryEco) and [here](https://github.com/Yadhu-S/UnrealJolt)

See [a list of projects that use AsterCore Physics here](Docs/ProjectsUsingJolt.md).

## License

The project is distributed under the [MIT license](LICENSE).

## Contributions

All contributions are welcome! If you intend to make larger changes, please discuss first in the GitHub Discussion section. For non-trivial changes, we require that you agree to a [Contributor Agreement](ContributorAgreement.md). When you create a PR, [CLA assistant](https://cla-assistant.io/) will prompt you to sign it.

Note that all PRs will be squashed before merging, so there's no need to force-push to git to keep the history clean.



