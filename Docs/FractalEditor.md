# Fractal Editor

Fractal Editor is the AsterCore 3.1.0 fracture authoring layer for destruction content creation.

Included SDK authoring concepts:

- Voronoi and clustered Voronoi fracture patterns
- Impact-point driven clustering for authored hit locations
- Material-based fracture presets
  - Wood prefers longer shards
  - Glass prefers tiny polygon-heavy breakup
  - Stone and concrete favor heavier chunking
- Static-to-dynamic transition thresholds for sleeping debris
- Simplified chunk hierarchy controls for far-field merge and cull behavior

The current SDK foundation is exposed through:

- `Tools/FractalEditor/FractureAuthoringProfile.*`
- `Tools/Editor/PhysicsAssetDocument.*`
- `AsterFractalEditorCLI`
