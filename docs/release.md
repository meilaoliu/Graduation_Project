# Release Guide

This guide documents the release workflow for the public OmniInspect repository.

## What Goes Into Git

Commit the ROS workspace source, launch files, maps, scripts, documentation, tests, and small benchmark artifacts.
Do not commit local build products, caches, API keys, the old nested workspace, or high-fidelity DAE assets.

Before publishing, run:

```bash
scripts/check_release_tree.sh
catkin build
source devel/setup.bash
python3 -m pytest src/benchmark src/nlp_commander/tests
```

## Asset Release

The high-fidelity substation model is distributed as a GitHub Release asset named:

```text
substation_dae_assets_v1.tar.gz
```

The download script expects this default URL:

```text
https://github.com/meilaoliu/OmniInspect/releases/download/assets-v1/substation_dae_assets_v1.tar.gz
```

Create the asset archive from the local ignored `substation_dae/` directory:

```bash
scripts/package_substation_assets.sh
```

The script writes the archive and a `.sha256` file to `/tmp/omniinspect-release/` by default.
It includes `model.config`, `model.sdf`, `meshes/substation.dae`, and `materials/textures/`.
It intentionally excludes local duplicate mesh files such as `substation(copy).dae`.

## Create a GitHub Release With the CLI

After the code has been committed and pushed:

```bash
git tag assets-v1
git push origin assets-v1

gh release create assets-v1 \
  /tmp/omniinspect-release/substation_dae_assets_v1.tar.gz \
  /tmp/omniinspect-release/substation_dae_assets_v1.tar.gz.sha256 \
  --title "Substation assets v1" \
  --notes "High-fidelity substation DAE mesh and texture assets for OmniInspect."
```

If the release already exists and you only need to upload or replace the asset:

```bash
gh release upload assets-v1 \
  /tmp/omniinspect-release/substation_dae_assets_v1.tar.gz \
  /tmp/omniinspect-release/substation_dae_assets_v1.tar.gz.sha256 \
  --clobber
```

## Create a GitHub Release in the Web UI

1. Open the GitHub repository page.
2. Click `Releases`.
3. Click `Draft a new release`.
4. Set the tag to `assets-v1`.
5. Set the title to `Substation assets v1`.
6. Upload `substation_dae_assets_v1.tar.gz` and the `.sha256` file.
7. Publish the release.

## Verify the Public Asset

After publishing the release, test from a clean clone:

```bash
git clone https://github.com/meilaoliu/OmniInspect.git
cd OmniInspect
catkin build
scripts/download_substation_assets.sh
test -f src/substation_description/models/substation_dae/meshes/substation.dae
```

If the default URL changes, update `ASSET_URL` in `scripts/download_substation_assets.sh` and the URL documented in `README.md` and `docs/assets.md`.
