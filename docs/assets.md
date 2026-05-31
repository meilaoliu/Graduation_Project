# Assets

The high-fidelity `substation_dae` mesh and textures are too large for normal Git tracking.
They are distributed as a GitHub Release asset instead of Git LFS.

Expected installed layout:

```text
src/substation_description/models/substation_dae/
├── model.config
├── model.sdf
├── meshes/substation.dae
└── materials/textures/
```

Install the asset:

```bash
scripts/download_substation_assets.sh
```

To use a mirror or a locally hosted archive:

```bash
OMNIINSPECT_SUBSTATION_ASSET_URL=https://example.com/substation_dae_assets_v1.tar.gz \
  scripts/download_substation_assets.sh
```

Supported archive formats are `.zip`, `.tar.gz`, and `.tar.zst`.
The archive may either contain a top-level `substation_dae/` directory or the model contents directly.
