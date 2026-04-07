# ACD (AsterCore Debugger)

ACD is the AsterCore 3.1.0 debugger layer for live and offline inspection of physics scenes. It is designed as the SDK-side counterpart to tools such as PhysX Visual Debugger.

Current SDK foundation:

- Frame capture session export through `Tools/ACD/ACDCaptureSession.*`
- CLI generation of `.json` capture sessions through `AsterCoreDebuggerCLI`
- Capture fields for frame time, memory, determinism checksum, counters, markers, and event logs
- Compute backend toggle support for switching between Vulkan Compute and DirectCompute in debugger captures, with explicit CPU fallback metadata

Typical uses:

- Compare deterministic rollback runs
- Inspect destruction spikes and GPU compute job timing
- Export captures from gameplay builds for offline diagnosis
- Validate Vulkan-vs-DirectCompute debugger runs across different GPU vendors with an explicit capture-side backend selection path
