AsterCore 3.1 Tools
=================

AsterCore 3.1 is packaged as an SDK-oriented release.
This folder contains tool-side components for debugging, asset authoring, and fracture authoring.

Tools included
--------------
1. AsterCoreDebuggerCLI
   - Target: AsterCoreDebuggerCLI
   - Support library: AsterCoreACDCore
   - Purpose: Capture debugger-friendly runtime data such as frame timing, counters, memory usage, and determinism checksums.

2. AsterPhysicsEditorCLI
   - Target: AsterPhysicsEditorCLI
   - Support library: AsterCoreEditorCore
   - Purpose: Build or export physics assets through the editor-side asset pipeline.

3. AsterFractalEditorCLI
   - Target: AsterFractalEditorCLI
   - Support library: AsterCoreFractalEditorCore
   - Purpose: Author destruction and fracture profiles such as Voronoi, clustered fracture, material-based fracture presets, and chunk hierarchy settings.

Build
-----
From the repository root, configure a build folder with CMake and build the desired tool targets.

Example with Ninja:

  cmake -S . -B build-tools -G Ninja
  cmake --build build-tools --target AsterCoreDebuggerCLI --parallel 4
  cmake --build build-tools --target AsterPhysicsEditorCLI --parallel 4
  cmake --build build-tools --target AsterFractalEditorCLI --parallel 4

If you want all three at once:

  cmake --build build-tools --target AsterCoreDebuggerCLI AsterPhysicsEditorCLI AsterFractalEditorCLI --parallel 4

Output
------
The executables are generated in the selected build directory, for example:

  build-tools\AsterCoreDebuggerCLI.exe
  build-tools\AsterPhysicsEditorCLI.exe
  build-tools\AsterFractalEditorCLI.exe

Usage
-----
AsterCoreDebuggerCLI
  Use this tool when you want an offline debugger capture for diagnostics or regression tracking.
  Typical output includes timing data, memory information, counters, markers, determinism-related checksums, and an explicit Vulkan-vs-DirectCompute backend toggle with CPU fallback metadata.

AsterPhysicsEditorCLI
  Use this tool to generate or export physics asset data for the runtime.
  This is the editor-side path for preparing assets that will later be consumed by AsterCore.

AsterFractalEditorCLI
  Use this tool to author fracture-related data for destruction workflows.
  This includes Voronoi and clustered fracture patterns, material presets such as wood, glass, concrete, or metal, wake thresholds, and simplified chunk hierarchy settings.

Recommended workflow
--------------------
1. Author or export base physics assets with AsterPhysicsEditorCLI.
2. Author fracture profiles with AsterFractalEditorCLI for destruction-heavy assets.
3. Capture runtime diagnostics with AsterCoreDebuggerCLI when validating performance, determinism, or memory behavior.

Notes
-----
- These tools build on top of the main AsterCore runtime and are intended for SDK and middleware workflows.
- For packaged SDK output, see the install and packaging files under Build\Packaging.
- Additional high-level documentation is available in Docs\ACD.md and Docs\FractalEditor.md.
