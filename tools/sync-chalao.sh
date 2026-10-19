#!/usr/bin/env bash
# Sync vendor/chalao with the Robot Chalao native core at a release tag.
#   tools/sync-chalao.sh v1.0.0          copy that tag's native/include into vendor/chalao
#   tools/sync-chalao.sh --check         fail if vendor/chalao differs from its pinned tag
set -euo pipefail
cd "$(dirname "$0")/.."
REPO=https://github.com/megazron/robot-chalao
check=0
if [ "${1:-}" = "--check" ]; then check=1; tag="$(cat vendor/chalao/VERSION)"; else tag="${1:?usage: $0 <tag> | --check}"; fi
tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
git clone -q --depth 1 --branch "$tag" "$REPO" "$tmp/rc"
if [ "$check" = 1 ]; then
  if diff -r -x VERSION "$tmp/rc/native/include" vendor/chalao; then
    echo "vendor/chalao matches robot-chalao $tag"
  else
    echo "vendor/chalao has drifted from robot-chalao $tag -- run tools/sync-chalao.sh <tag>"; exit 1
  fi
else
  cp "$tmp/rc/native/include/"*.hpp vendor/chalao/
  echo "$tag" > vendor/chalao/VERSION
  echo "vendor/chalao <- robot-chalao $tag; now run: make test"
fi
