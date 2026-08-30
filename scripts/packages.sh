#!/usr/bin/env bash
# Build all radapter DEB packages and the ROS plugin .so files.
# Output lands in ${OUT:-out}/.

set -eu

OUT="${OUT:-out}"
mkdir -p "$OUT"

docker buildx build -f scripts/Dockerfile.cross --target pkg  --output="$OUT" .

echo "=== done ==="
ls -la "$OUT"
