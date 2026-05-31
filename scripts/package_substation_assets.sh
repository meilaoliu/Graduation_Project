#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LOCAL_ASSET_DIR="${1:-$WORKSPACE_ROOT/substation_dae}"
OUTPUT_DIR="${2:-/tmp/omniinspect-release}"
OUTPUT_NAME="${3:-substation_dae_assets_v1.tar.gz}"

MODEL_DIR="$WORKSPACE_ROOT/src/substation_description/models/substation_dae"
STAGE_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "$STAGE_DIR"
}
trap cleanup EXIT

if [ ! -f "$MODEL_DIR/model.config" ] || [ ! -f "$MODEL_DIR/model.sdf" ]; then
  echo "Missing model metadata in $MODEL_DIR" >&2
  exit 1
fi

if [ ! -f "$LOCAL_ASSET_DIR/meshes/substation.dae" ]; then
  echo "Missing mesh: $LOCAL_ASSET_DIR/meshes/substation.dae" >&2
  echo "Pass the local asset directory as the first argument if it is stored elsewhere." >&2
  exit 1
fi

mkdir -p "$STAGE_DIR/substation_dae/meshes" \
  "$STAGE_DIR/substation_dae/materials/textures" \
  "$OUTPUT_DIR"

cp "$MODEL_DIR/model.config" "$STAGE_DIR/substation_dae/model.config"
cp "$MODEL_DIR/model.sdf" "$STAGE_DIR/substation_dae/model.sdf"
cp "$LOCAL_ASSET_DIR/meshes/substation.dae" "$STAGE_DIR/substation_dae/meshes/substation.dae"

if [ -d "$LOCAL_ASSET_DIR/materials/textures" ]; then
  cp -a "$LOCAL_ASSET_DIR/materials/textures/." "$STAGE_DIR/substation_dae/materials/textures/"
fi

case "$OUTPUT_NAME" in
  *.tar.gz|*.tgz)
    tar -czf "$OUTPUT_DIR/$OUTPUT_NAME" -C "$STAGE_DIR" substation_dae
    ;;
  *.tar.zst|*.tzst)
    command -v zstd >/dev/null || {
      echo "zstd is required to create $OUTPUT_NAME" >&2
      exit 1
    }
    tar --zstd -cf "$OUTPUT_DIR/$OUTPUT_NAME" -C "$STAGE_DIR" substation_dae
    ;;
  *)
    echo "Unsupported output archive type: $OUTPUT_NAME" >&2
    echo "Use .tar.gz or .tar.zst." >&2
    exit 1
    ;;
esac
sha256sum "$OUTPUT_DIR/$OUTPUT_NAME" > "$OUTPUT_DIR/$OUTPUT_NAME.sha256"

echo "Wrote:"
echo "  $OUTPUT_DIR/$OUTPUT_NAME"
echo "  $OUTPUT_DIR/$OUTPUT_NAME.sha256"
