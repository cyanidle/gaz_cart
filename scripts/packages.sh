#!/usr/bin/env bash
# Build the unified Bookworm arm64 gaz-cart DEB (requires arm64 binfmt).
# Output lands in ${OUT:-out}/.

set -eu

OUT="${OUT:-out}"
mkdir -p "$OUT"

docker buildx build -f scripts/Dockerfile.cross --target pkg --output="$OUT" .

echo "=== done ==="
ls -la "$OUT"
