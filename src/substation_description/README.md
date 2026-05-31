# substation_description

This package provides the Gazebo model layout for the high-fidelity OmniInspect substation scene.

The repository tracks only the small Gazebo metadata files under `models/substation_dae/`.
The large DAE mesh and textures are distributed as release assets and should be installed with:

```bash
scripts/download_substation_assets.sh
```

After download, the expected layout is:

```text
src/substation_description/models/substation_dae/
├── model.config
├── model.sdf
├── meshes/substation.dae
└── materials/textures/
```
