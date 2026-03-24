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

## AsterCore 2.1 Highlights

1. AsterCore 2.1 broadens the runtime beyond the core rigid body stack. In addition to the existing collision, character, vehicle and soft body systems, this branch now includes destruction tooling for pre-fractured assets, a snapshot foundation for rollback and replay workflows, and a dedicated hair module for strand-oriented simulation utilities.

2. Several gameplay-facing systems have been expanded so the engine is easier to use in real projects. Vehicles now cover wheeled, tracked and motorcycle controllers, water interaction helpers are part of the runtime, secondary motion can be handled through JiggleConstraint, and particle fluid simulation is available for scenes that need lightweight fluid-body interaction.

3. The project is also in a better place as middleware than earlier branches. The CMake setup supports building AsterCore as either a static or shared library, and the README now documents the shared-library path more clearly for engine integration work such as Feliss.

4. Tooling around the runtime has started to catch up with the feature set. This tree includes editor-side asset helpers, integration scaffolding for external engines, Python/C API entry points, and snapshot-aware systems that make it easier to move from isolated physics tests toward full production workflows.

## Supported platforms

* Windows (Desktop or UWP) x86/x64/ARM32/ARM64
* Linux (tested on Ubuntu) x86/x64/ARM32/ARM64/RISC-V64/LoongArch64/PowerPC64LE
* FreeBSD
* Android x86/x64/ARM32/ARM64
* Platform Blue (a popular game console) x64
* macOS x64/ARM64
* iOS x64/ARM64
* MSYS2 MinGW64
* WebAssembly, see [this](https://github.com/jrouwe/JoltPhysics.js) separate project.

## Required CPU features

* On x86/x64 the minimal requirements are SSE2. The library can be compiled using SSE4.1, SSE4.2, AVX, AVX2, or AVX512.
* On ARM64 the library uses NEON and FP16. On ARM32 it can be compiled without any special CPU instructions.

## Documentation

To get started, look at the [HelloWorld](HelloWorld/HelloWorld.cpp) example. A [HelloWorld example using CMake FetchContent](https://github.com/jrouwe/JoltPhysicsHelloWorld) is also available to show how you can integrate AsterCore Physics in a CMake project.

Every feature in AsterCore has its own sample. [Running the Samples application](Docs/Samples.md) and browsing through the [code](https://github.com/jrouwe/JoltPhysics/tree/master/Samples/Tests) is a great way to learn about the library!

To learn more about AsterCore go to the latest [Architecture and API documentation](https://jrouwe.github.io/JoltPhysics/). Documentation for [a specific release is also available](https://jrouwe.github.io/JoltPhysicsDocs/).

Some algorithms used by AsterCore are described in detail in my GDC 2022 talk: Architecting AsterCore Physics for 'Horizon Forbidden West' ([slides](https://gdcvault.com/play/1027560/Architecting-AsterCore-Physics-for-Horizon), [slides with speaker notes](https://jrouwe.nl/architectingjolt/ArchitectingJoltPhysics_Rouwe_Jorrit_Notes.pdf), [video](https://gdcvault.com/play/1027891/Architecting-AsterCore-Physics-for-Horizon)).

## Compiling

* Compiles with Visual Studio 2019+, Clang 10+ or GCC 9+.
* Uses C++ 17.
* Depends only on the standard template library.
* Doesn't use RTTI.
* Doesn't use exceptions.

If you want to run on Platform Blue you'll need to provide your own build environment and PlatformBlue.h due to NDA requirements. This file is available on the Platform Blue developer forum.

For build instructions go to the [Build](Build/README.md) section. When upgrading from an older version of the library go to the [Release Notes](Docs/ReleaseNotes.md) or [API Changes](Docs/APIChanges.md) sections.

### Building the shared library for Feliss integration

The project supports building AsterCore Physics 2.1.0 as a shared library.

#### Windows (MSYS2 MinGW64 + Ninja)

Configure and build `AsterCore.dll`:

```powershell
cmake -S . -B build-aster-shared -G Ninja `
  -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe `
  -DBUILD_SHARED_LIBS=ON `
  -DBUILD_ASTERCORE_UNIT_TESTS=OFF `
  -DBUILD_ASTERCORE_SAMPLES=OFF `
  -DBUILD_ASTERCORE_PERFORMANCE_TEST=OFF `
  -DBUILD_ASTERCORE_VIEWER=OFF `
  -DBUILD_ASTERCORE_BINDINGS=OFF `
  -DBUILD_ASTERCORE_EDITOR_TOOLS=OFF `
  -DBUILD_ASTERCORE_INTEGRATIONS=OFF

cmake --build build-aster-shared --target AsterCore
```

Output files:

- `build-aster-shared/AsterCore.dll`
- `build-aster-shared/libAsterCore.dll.a`

If Feliss links dynamically, ship `AsterCore.dll` next to the Feliss executable or in a directory that is part of the runtime search path.

#### Linux (GCC or Clang + Ninja)

Configure and build `libAsterCore.so`:

```bash
cmake -S . -B build-aster-shared -G Ninja \
  -DCMAKE_CXX_COMPILER=g++ \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_ASTERCORE_UNIT_TESTS=OFF \
  -DBUILD_ASTERCORE_SAMPLES=OFF \
  -DBUILD_ASTERCORE_PERFORMANCE_TEST=OFF \
  -DBUILD_ASTERCORE_VIEWER=OFF \
  -DBUILD_ASTERCORE_BINDINGS=OFF \
  -DBUILD_ASTERCORE_EDITOR_TOOLS=OFF \
  -DBUILD_ASTERCORE_INTEGRATIONS=OFF

cmake --build build-aster-shared --target AsterCore
```

Typical Linux output files:

- `build-aster-shared/libAsterCore.so`
- optional symlink/versioned `.so` files depending on generator and toolchain

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



