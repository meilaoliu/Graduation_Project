#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_DIR="$WORKSPACE_ROOT/src/substation_description/models/substation_dae"

ASSET_URL="${OMNIINSPECT_SUBSTATION_ASSET_URL:-https://github.com/meilaoliu/OmniInspect/releases/download/assets-v1/substation_dae_assets_v1.tar.gz}"
ASSET_SHA256="${OMNIINSPECT_SUBSTATION_ASSET_SHA256:-}"

TMP_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

case "$ASSET_URL" in
  *.zip) ARCHIVE="$TMP_DIR/substation_dae_assets.zip" ;;
  *.tar.gz|*.tgz) ARCHIVE="$TMP_DIR/substation_dae_assets.tar.gz" ;;
  *.tar.zst|*.tzst) ARCHIVE="$TMP_DIR/substation_dae_assets.tar.zst" ;;
  *) ARCHIVE="$TMP_DIR/substation_dae_assets.archive" ;;
esac

echo "Downloading substation assets:"
echo "  $ASSET_URL"
curl -L "$ASSET_URL" -o "$ARCHIVE"

if [ -n "$ASSET_SHA256" ]; then
  echo "$ASSET_SHA256  $ARCHIVE" | sha256sum -c -
fi

EXTRACT_DIR="$TMP_DIR/extract"
mkdir -p "$EXTRACT_DIR" "$TARGET_DIR"

case "$ARCHIVE" in
  *.zip)
    command -v unzip >/dev/null || {
      echo "unzip is required to extract $ARCHIVE" >&2
      exit 1
    }
    unzip -q "$ARCHIVE" -d "$EXTRACT_DIR"
    ;;
  *.tar.gz)
    tar -xzf "$ARCHIVE" -C "$EXTRACT_DIR"
    ;;
  *.tar.zst)
    tar --zstd -xf "$ARCHIVE" -C "$EXTRACT_DIR"
    ;;
  *)
    echo "Unsupported archive type: $ARCHIVE" >&2
    exit 1
    ;;
esac

if [ -d "$EXTRACT_DIR/substation_dae" ]; then
  cp -a "$EXTRACT_DIR/substation_dae/." "$TARGET_DIR/"
else
  cp -a "$EXTRACT_DIR/." "$TARGET_DIR/"
fi

if [ ! -f "$TARGET_DIR/meshes/substation.dae" ]; then
  echo "Expected mesh not found: $TARGET_DIR/meshes/substation.dae" >&2
  exit 1
fi

echo "Substation assets installed at:"
echo "  $TARGET_DIR"
