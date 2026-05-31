#!/usr/bin/env bash
set -euo pipefail

echo "Checking tracked release tree..."

bad_tracked_paths="$(git ls-files | rg '(^|/)(build|devel|logs|\.catkin_tools|\.pytest_cache|\.vscode|\.idea|__pycache__)(/|$)|\.log$' || true)"
if [ -n "$bad_tracked_paths" ]; then
  echo "Generated or local-only paths are tracked:" >&2
  echo "$bad_tracked_paths" >&2
  exit 1
fi

large_files="$(git ls-files -z | xargs -0 -I{} sh -c '[ -f "$1" ] && [ "$(wc -c < "$1")" -gt 94371840 ] && printf "%s\n" "$1"' sh {} || true)"
if [ -n "$large_files" ]; then
  echo "Tracked files larger than 90 MiB:" >&2
  echo "$large_files" >&2
  exit 1
fi

hardcoded_paths="$(rg -n '/home/leo|Graduation_Project/ego-planner-for-ground-robot' README.md docs src .github 2>/dev/null || true)"
if [ -n "$hardcoded_paths" ]; then
  echo "Hardcoded local paths remain:" >&2
  echo "$hardcoded_paths" >&2
  exit 1
fi

echo "Release tree checks passed."
