#!/usr/bin/env bash
# Build all cart-stack packages for one architecture and drop them into
# ${OUT:-out}/:
#   radapter-jit-headless_<ver>_<arch>.deb — the engine, LuaJIT runtime
#   radapter-ros_<ver>_<arch>.deb          — the ROS2 bridge plugin (Jazzy deps)
#   gaz-cart_<ver>_<arch>.deb              — the merged native plugins
# The Lua cart runtime is not packaged: deploy it from a repository checkout.
#
# All three are built for the LuaJIT runtime so their DEB relations match and
# `dpkg -i` accepts them together.
#
# Usage: packages.sh [x64|arm64]   (default: arm64)
#
# arm64 cross-builds need QEMU binfmt for the sysroot stages:
#   docker run --privileged --rm tonistiigi/binfmt --install arm64

set -eu

ARCH="${1:-arm64}"
OUT="${OUT:-out}"
BUILD_JOBS="${BUILD_JOBS:-4}"

case "$ARCH" in
    arm64)
        radapter_df=cross
        ros_target=cross-deb-pkg
        gaz_df=cross
        ;;
    x64)
        radapter_df=native
        ros_target=native-deb-pkg
        gaz_df=native
        ;;
    *)
        echo "usage: $0 [x64|arm64]" >&2
        exit 1
        ;;
esac

mkdir -p "$OUT"

echo "=== radapter-jit-headless DEB ($ARCH) ==="
docker buildx build -f "radapter/scripts/Dockerfile.$radapter_df" \
    --build-arg RADAPTER_JIT=ON --build-arg BUILD_JOBS="$BUILD_JOBS" \
    --target headless-pkg --output="$OUT" radapter

echo "=== radapter-ros DEB ($ARCH) ==="
docker buildx build -f radapter/scripts/Dockerfile.ros \
    --build-arg RADAPTER_JIT=ON --build-arg BUILD_JOBS="$BUILD_JOBS" \
    --target "$ros_target" --output="$OUT" radapter

echo "=== gaz-cart DEB ($ARCH) ==="
docker buildx build -f "scripts/Dockerfile.$gaz_df" \
    --build-arg BUILD_JOBS="$BUILD_JOBS" --target pkg --output="$OUT" .

echo "=== done ==="
ls -la "$OUT"
